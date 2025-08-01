#include "detection_controller.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <algorithm>

DetectionController::DetectionController(const DetectionConfig& config)
    : state_(DetectionState::DISABLED)
    , threshold_(config.defaultThreshold)
    , autoServoAction_(config.autoServoAction)
    , cooldownMs_(config.cooldownMs)
    , stopEventThread_(false)
    , loggingEnabled_(config.enableLogging)
    , logFilePath_(config.logFilePath) {
    
    stats_.startTime = std::chrono::system_clock::now();
    
    // 이벤트 처리 스레드 시작
    eventThread_ = std::thread(&DetectionController::eventThreadFunc, this);
    
    // 로그 파일 열기
    if (loggingEnabled_) {
        std::lock_guard<std::mutex> lock(logMutex_);
        logFile_.open(logFilePath_, std::ios::app);
        if (logFile_.is_open()) {
            logFile_ << "\n=== Detection Controller Started ===" << std::endl;
            logFile_ << "Time: " << std::chrono::system_clock::now().time_since_epoch().count() << std::endl;
            logFile_ << "Default Threshold: " << threshold_ << std::endl;
            logFile_ << "====================================" << std::endl;
        }
    }
}

DetectionController::~DetectionController() {
    // 이벤트 스레드 종료
    stopEventThread_ = true;
    eventCv_.notify_all();
    if (eventThread_.joinable()) {
        eventThread_.join();
    }
    
    // 로그 파일 닫기
    if (logFile_.is_open()) {
        logFile_ << "=== Detection Controller Stopped ===" << std::endl;
        logFile_.close();
    }
}

void DetectionController::enable() {
    if (state_ == DetectionState::DISABLED) {
        setState(DetectionState::ENABLED);
        pushEvent({DetectionEvent::STATE_CHANGED, std::chrono::system_clock::now(), 
                  0, threshold_, "Detection enabled"});
    }
}

void DetectionController::disable() {
    setState(DetectionState::DISABLED);
    pushEvent({DetectionEvent::STATE_CHANGED, std::chrono::system_clock::now(), 
              0, threshold_, "Detection disabled"});
}

void DetectionController::arm() {
    if (state_ == DetectionState::ENABLED) {
        setState(DetectionState::ARMED);
        pushEvent({DetectionEvent::STATE_CHANGED, std::chrono::system_clock::now(), 
                  0, threshold_, "Detection armed - servo action enabled"});
    }
}

void DetectionController::disarm() {
    if (state_ == DetectionState::ARMED || state_ == DetectionState::TRIGGERED) {
        setState(DetectionState::ENABLED);
        pushEvent({DetectionEvent::STATE_CHANGED, std::chrono::system_clock::now(), 
                  0, threshold_, "Detection disarmed - servo action disabled"});
    }
}

bool DetectionController::processCommand(const std::string& command) {
    if (command == "d_on") {
        arm();
        return true;
    } else if (command == "d_off") {
        disarm();
        return true;
    } else if (command == "enable") {
        enable();
        return true;
    } else if (command == "disable") {
        disable();
        return true;
    }
    
    // 임계값 설정 명령 처리
    if (command.substr(0, 10) == "threshold:") {
        try {
            double newThreshold = std::stod(command.substr(10));
            setThreshold(newThreshold);
            return true;
        } catch (...) {
            return false;
        }
    }
    
    return false;
}

// detection_controller.cpp 파일

bool DetectionController::shouldTriggerServo(double sadValue) {
    updateStats(sadValue);

    // ========================>   수정된 부분 시작   <========================
    // 쿨다운 상태인지 확인하고, 시간이 만료되었으면 자동으로 ARMED 상태로 복귀시킵니다.
    // 이 로직은 reportPass()를 외부에서 호출할 필요를 없애줍니다.
    if (state_ == DetectionState::COOLDOWN) {
        if (std::chrono::system_clock::now() >= cooldownEndTime_) {
            setState(DetectionState::ARMED);
            pushEvent({DetectionEvent::BOTTLE_PASSED, std::chrono::system_clock::now(),
                      0, threshold_, "Cooldown finished, system re-armed"});
        }
    }
    // ========================>    수정된 부분 끝    <========================

    // ARMED 상태가 아니면(COOLDOWN 포함) 서보를 동작시키지 않습니다.
    if (state_ != DetectionState::ARMED) {
        return false;
    }

    // 임계값 확인
    if (sadValue <= threshold_) {
        return false;
    }

    // 감지 트리거
    setState(DetectionState::TRIGGERED);
    lastTriggerTime_ = std::chrono::system_clock::now();
    cooldownEndTime_ = lastTriggerTime_ + std::chrono::milliseconds(cooldownMs_);

    {
        std::lock_guard<std::mutex> lock(statsMutex_);
        stats_.totalDetections++;
        stats_.lastDetection = lastTriggerTime_;
        if (autoServoAction_) {
            stats_.servoActions++;
        }
    }

    pushEvent({DetectionEvent::BOTTLE_DETECTED, lastTriggerTime_,
              sadValue, threshold_, "Bottle detected - trigger servo"});

    if (autoServoAction_) {
        pushEvent({DetectionEvent::SERVO_ACTION, lastTriggerTime_,
                  sadValue, threshold_, "Servo action triggered"});
    }

    // 쿨다운 상태로 전환
    setState(DetectionState::COOLDOWN);

    return autoServoAction_;
}

void DetectionController::reportDetection(double sadValue) {
    updateStats(sadValue);
    
    if (state_ == DetectionState::ENABLED && sadValue > threshold_) {
        // 서보 동작 없이 감지만 기록
        std::lock_guard<std::mutex> lock(statsMutex_);
        stats_.totalDetections++;
        stats_.lastDetection = std::chrono::system_clock::now();
        
        pushEvent({DetectionEvent::BOTTLE_DETECTED, stats_.lastDetection, 
                  sadValue, threshold_, "Bottle detected - no servo action"});
    }
}

void DetectionController::reportPass() {
    if (state_ == DetectionState::COOLDOWN) {
        auto now = std::chrono::system_clock::now();
        if (now >= cooldownEndTime_) {
            setState(DetectionState::ARMED);
            pushEvent({DetectionEvent::BOTTLE_PASSED, now, 
                      0, threshold_, "Bottle passed - ready for next"});
        }
    }
}

void DetectionController::setThreshold(double threshold) {
    if (threshold > 0) {
        threshold_ = threshold;
        pushEvent({DetectionEvent::THRESHOLD_CHANGED, std::chrono::system_clock::now(), 
                  0, threshold, "Threshold changed"});
    }
}

double DetectionController::getThreshold() const {
    return threshold_.load();
}

void DetectionController::setAutoServoAction(bool enable) {
    autoServoAction_ = enable;
}

bool DetectionController::isAutoServoActionEnabled() const {
    return autoServoAction_.load();
}

void DetectionController::setCooldownMs(int ms) {
    if (ms > 0) {
        cooldownMs_ = ms;
    }
}

DetectionState DetectionController::getState() const {
    return state_.load();
}

std::string DetectionController::getStateString() const {
    switch (state_.load()) {
        case DetectionState::DISABLED: return "DISABLED";
        case DetectionState::ENABLED: return "ENABLED";
        case DetectionState::ARMED: return "ARMED";
        case DetectionState::TRIGGERED: return "TRIGGERED";
        case DetectionState::COOLDOWN: return "COOLDOWN";
        default: return "UNKNOWN";
    }
}

bool DetectionController::isEnabled() const {
    DetectionState s = state_.load();
    return s != DetectionState::DISABLED;
}

bool DetectionController::isArmed() const {
    DetectionState s = state_.load();
    return s == DetectionState::ARMED || s == DetectionState::TRIGGERED || s == DetectionState::COOLDOWN;
}

DetectionStats DetectionController::getStats() const {
    std::lock_guard<std::mutex> lock(statsMutex_);
    return stats_;
}

void DetectionController::resetStats() {
    std::lock_guard<std::mutex> lock(statsMutex_);
    stats_ = DetectionStats();
    stats_.startTime = std::chrono::system_clock::now();
    sadHistory_.clear();
}

void DetectionController::setEventHandler(EventHandler handler) {
    eventHandler_ = handler;
}

void DetectionController::enableLogging(bool enable) {
    loggingEnabled_ = enable;
    if (enable && !logFile_.is_open()) {
        std::lock_guard<std::mutex> lock(logMutex_);
        logFile_.open(logFilePath_, std::ios::app);
    } else if (!enable && logFile_.is_open()) {
        std::lock_guard<std::mutex> lock(logMutex_);
        logFile_.close();
    }
}

void DetectionController::setLogFile(const std::string& path) {
    std::lock_guard<std::mutex> lock(logMutex_);
    if (logFile_.is_open()) {
        logFile_.close();
    }
    logFilePath_ = path;
    if (loggingEnabled_) {
        logFile_.open(logFilePath_, std::ios::app);
    }
}

void DetectionController::setState(DetectionState newState) {
    DetectionState oldState = state_.exchange(newState);
    if (oldState != newState) {
        std::cout << "State transition: " << getStateString() << std::endl;
    }
}

void DetectionController::updateStats(double sadValue) {
    std::lock_guard<std::mutex> lock(statsMutex_);
    
    // 히스토리 업데이트
    sadHistory_.push_back(sadValue);
    if (sadHistory_.size() > HISTORY_SIZE) {
        sadHistory_.pop_front();
    }
    
    // 통계 업데이트
    if (sadValue > stats_.maxSAD) {
        stats_.maxSAD = sadValue;
    }
    if (sadValue < stats_.minSAD) {
        stats_.minSAD = sadValue;
    }
    
    // 평균 계산
    if (!sadHistory_.empty()) {
        double sum = 0;
        for (double val : sadHistory_) {
            sum += val;
        }
        stats_.averageSAD = sum / sadHistory_.size();
    }
}

void DetectionController::pushEvent(const DetectionEventData& event) {
    {
        std::lock_guard<std::mutex> lock(eventMutex_);
        eventQueue_.push(event);
    }
    eventCv_.notify_one();
    
    // 로깅
    if (loggingEnabled_) {
        logEvent(event);
    }
}

void DetectionController::eventThreadFunc() {
    while (!stopEventThread_) {
        std::unique_lock<std::mutex> lock(eventMutex_);
        eventCv_.wait(lock, [this] { return !eventQueue_.empty() || stopEventThread_; });
        
        while (!eventQueue_.empty()) {
            DetectionEventData event = eventQueue_.front();
            eventQueue_.pop();
            lock.unlock();
            
            // 이벤트 핸들러 호출
            if (eventHandler_) {
                eventHandler_(event);
            }
            
            lock.lock();
        }
    }
}

void DetectionController::logEvent(const DetectionEventData& event) {
    std::lock_guard<std::mutex> lock(logMutex_);
    if (logFile_.is_open()) {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        
        logFile_ << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
        logFile_ << " | " << std::setw(20) << std::left;
        
        switch (event.event) {
            case DetectionEvent::BOTTLE_DETECTED:
                logFile_ << "BOTTLE_DETECTED";
                break;
            case DetectionEvent::BOTTLE_PASSED:
                logFile_ << "BOTTLE_PASSED";
                break;
            case DetectionEvent::THRESHOLD_CHANGED:
                logFile_ << "THRESHOLD_CHANGED";
                break;
            case DetectionEvent::STATE_CHANGED:
                logFile_ << "STATE_CHANGED";
                break;
            case DetectionEvent::SERVO_ACTION:
                logFile_ << "SERVO_ACTION";
                break;
        }
        
        logFile_ << " | SAD: " << std::fixed << std::setprecision(2) << event.sadValue;
        logFile_ << " | Threshold: " << event.threshold;
        logFile_ << " | " << event.message << std::endl;
    }
}

bool DetectionController::isInCooldown() const {
    if (state_ != DetectionState::COOLDOWN) {
        return false;
    }
    return std::chrono::system_clock::now() < cooldownEndTime_;
}