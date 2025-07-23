#ifndef DETECTION_CONTROLLER_H
#define DETECTION_CONTROLLER_H

#include <atomic>
#include <mutex>
#include <chrono>
#include <string>
#include <functional>
#include <thread>
#include <condition_variable>
#include <queue>
#include <memory>
#include <fstream>

// 감지 이벤트 타입
enum class DetectionEvent {
    BOTTLE_DETECTED,
    BOTTLE_PASSED,
    THRESHOLD_CHANGED,
    STATE_CHANGED,
    SERVO_ACTION
};

// 감지 상태
enum class DetectionState {
    DISABLED,           // 감지 비활성화
    ENABLED,           // 감지 활성화 
    ARMED,             // 감지 대기 (서보 동작 가능)
    TRIGGERED,         // 감지됨 (서보 동작 중)
    COOLDOWN          // 쿨다운 (다음 감지 대기)
};

// 감지 이벤트 데이터
struct DetectionEventData {
    DetectionEvent event;
    std::chrono::system_clock::time_point timestamp;
    double sadValue;
    double threshold;
    std::string message;
};

// 감지 제어 설정
struct DetectionConfig {
    double defaultThreshold = 500000.0;
    int cooldownMs = 1000;
    bool autoServoAction = true;
    bool enableLogging = true;
    std::string logFilePath = "/tmp/detection_log.txt";
};

// 감지 통계
struct DetectionStats {
    unsigned long totalDetections = 0;
    unsigned long servoActions = 0;
    unsigned long falsePositives = 0;
    std::chrono::system_clock::time_point lastDetection;
    std::chrono::system_clock::time_point startTime;
    double averageSAD = 0.0;
    double maxSAD = 0.0;
    double minSAD = std::numeric_limits<double>::max();
};

// 감지 제어 클래스
class DetectionController {
public:
    DetectionController(const DetectionConfig& config = DetectionConfig());
    ~DetectionController();

    // 상태 제어
    void enable();
    void disable();
    void arm();
    void disarm();
    bool processCommand(const std::string& command);

    // 감지 처리
    bool shouldTriggerServo(double sadValue);
    void reportDetection(double sadValue);
    void reportPass();

    // 설정
    void setThreshold(double threshold);
    double getThreshold() const;
    void setAutoServoAction(bool enable);
    bool isAutoServoActionEnabled() const;
    void setCooldownMs(int ms);

    // 상태 조회
    DetectionState getState() const;
    std::string getStateString() const;
    bool isEnabled() const;
    bool isArmed() const;

    // 통계
    DetectionStats getStats() const;
    void resetStats();

    // 이벤트 핸들러
    using EventHandler = std::function<void(const DetectionEventData&)>;
    void setEventHandler(EventHandler handler);

    // 로깅
    void enableLogging(bool enable);
    void setLogFile(const std::string& path);

private:
    // 내부 상태
    std::atomic<DetectionState> state_;
    std::atomic<double> threshold_;
    std::atomic<bool> autoServoAction_;
    std::atomic<int> cooldownMs_;
    
    // 통계
    mutable std::mutex statsMutex_;
    DetectionStats stats_;
    std::deque<double> sadHistory_;
    static constexpr size_t HISTORY_SIZE = 100;

    // 이벤트 처리
    EventHandler eventHandler_;
    std::queue<DetectionEventData> eventQueue_;
    std::thread eventThread_;
    std::condition_variable eventCv_;
    std::mutex eventMutex_;
    std::atomic<bool> stopEventThread_;

    // 쿨다운 관리
    std::chrono::system_clock::time_point lastTriggerTime_;
    std::chrono::system_clock::time_point cooldownEndTime_;

    // 로깅
    std::atomic<bool> loggingEnabled_;
    std::string logFilePath_;
    std::ofstream logFile_;
    std::mutex logMutex_;

    // 내부 메서드
    void setState(DetectionState newState);
    void updateStats(double sadValue);
    void pushEvent(const DetectionEventData& event);
    void eventThreadFunc();
    void logEvent(const DetectionEventData& event);
    bool isInCooldown() const;
};

// 싱글톤 인스턴스 (선택적)
class DetectionControllerSingleton {
public:
    static DetectionController& getInstance() {
        static DetectionController instance;
        return instance;
    }

private:
    DetectionControllerSingleton() = default;
    ~DetectionControllerSingleton() = default;
    DetectionControllerSingleton(const DetectionControllerSingleton&) = delete;
    DetectionControllerSingleton& operator=(const DetectionControllerSingleton&) = delete;
};

#endif // DETECTION_CONTROLLER_H