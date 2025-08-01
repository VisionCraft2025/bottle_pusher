#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <iomanip>
#include <chrono>
#include <deque>
#include <signal.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include "tcp_server.h"
#include "detection_controller.h"

// --- 전역 변수 선언 ---
std::vector<cv::Point> g_points;
cv::Mat g_currentFrame;
bool g_drawing = false;
cv::Mat g_roiMask;
bool g_roiSelected = false;

// 서보 모터 관련
int g_servoFd = -1;
std::mutex g_servoMutex;
bool g_servoEnabled = true;

// Detection Controller
std::unique_ptr<DetectionController> g_detectionController;

// TCP 서버 객체
TCPServer g_tcpServer;

// 성능 최적화를 위한 변수
const int CAPTURE_WIDTH = 320;
const int CAPTURE_HEIGHT = 240;
const int CAPTURE_FPS = 30;
const int BLUR_SIZE = 5;

// SAD 감지 관련
cv::Mat g_prevFrame;
cv::Mat g_baselineFrame;
// const int DEBOUNCE_FRAMES = 15;
// int g_framesSinceDetection = 0;
// bool g_isBottlePresent = false;

// 통계 및 성능 측정
double g_currentSAD = 0.0;
std::deque<double> g_sadHistory;
const int HISTORY_SIZE = 30;
double g_fps = 0.0;
auto g_lastTime = std::chrono::high_resolution_clock::now();
int g_frameCounter = 0;

// 멀티스레딩
std::atomic<bool> g_shouldExit(false);
cv::VideoCapture *g_cap = nullptr;

// 시그널 핸들러
void signalHandler(int signum)
{
    std::cout << "\n인터럽트 신호 받음. 프로그램을 종료합니다..." << std::endl;
    g_shouldExit = true;
}

// --- 서보 모터 관련 함수 ---
bool openServoDevice()
{
    std::lock_guard<std::mutex> lock(g_servoMutex);

    g_servoFd = open("/dev/servo_pusher", O_RDWR);
    if (g_servoFd < 0)
    {
        std::cerr << "서보 디바이스 열기 실패: " << strerror(errno) << std::endl;
        std::cerr << "드라이버가 로드되었는지 확인하세요:" << std::endl;
        std::cerr << "  sudo make load (드라이버 디렉토리에서)" << std::endl;
        return false;
    }

    std::cout << "서보 디바이스 연결 성공!" << std::endl;
    return true;
}

void closeServoDevice()
{
    std::lock_guard<std::mutex> lock(g_servoMutex);
    if (g_servoFd >= 0)
    {
        close(g_servoFd);
        g_servoFd = -1;
        std::cout << "서보 디바이스 연결 해제" << std::endl;
    }
}

bool sendServoCommand(const std::string &command)
{
    std::lock_guard<std::mutex> lock(g_servoMutex);

    if (g_servoFd < 0)
    {
        std::cerr << "서보 디바이스가 열려있지 않습니다." << std::endl;
        return false;
    }

    ssize_t written = write(g_servoFd, command.c_str(), command.length());
    if (written < 0)
    {
        std::cerr << "서보 명령 전송 실패: " << strerror(errno) << std::endl;
        return false;
    }

    return true;
}

std::string readServoStatus()
{
    std::lock_guard<std::mutex> lock(g_servoMutex);

    if (g_servoFd < 0)
    {
        return "서보 디바이스 연결 안됨";
    }

    char buffer[512];
    memset(buffer, 0, sizeof(buffer));

    lseek(g_servoFd, 0, SEEK_SET);

    ssize_t bytesRead = read(g_servoFd, buffer, sizeof(buffer) - 1);
    if (bytesRead < 0)
    {
        return "서보 상태 읽기 실패: " + std::string(strerror(errno));
    }

    return std::string(buffer, bytesRead);
}

void configureServo(int pushAngle, int restAngle, int duration)
{
    std::stringstream ss;

    ss.str("");
    ss << "push_angle:" << pushAngle;
    if (sendServoCommand(ss.str()))
    {
        std::cout << "서보 밀기 각도 설정: " << pushAngle << "도" << std::endl;
    }

    ss.str("");
    ss << "rest_angle:" << restAngle;
    if (sendServoCommand(ss.str()))
    {
        std::cout << "서보 대기 각도 설정: " << restAngle << "도" << std::endl;
    }

    ss.str("");
    ss << "duration:" << duration;
    if (sendServoCommand(ss.str()))
    {
        std::cout << "서보 동작 시간 설정: " << duration << "ms" << std::endl;
    }
}

// Detection Controller 이벤트 핸들러
void detectionEventHandler(const DetectionEventData &event)
{
    switch (event.event)
    {
    case DetectionEvent::SERVO_ACTION:
        // 서보 동작 실행
        if (g_servoEnabled && g_servoFd >= 0)
        {
            sendServoCommand("push");
        }
        break;

    case DetectionEvent::STATE_CHANGED:
        std::cout << "[STATE] " << event.message << std::endl;
        break;

    case DetectionEvent::THRESHOLD_CHANGED:
        std::cout << "[THRESHOLD] New threshold: " << event.threshold << std::endl;
        break;

    default:
        break;
    }
}

// --- 함수 선언 ---
void onMouse(int event, int x, int y, int flags, void *userdata);
void pushBottle();
void inputHandler();
void captureBaseline();
double calculateFastSAD(const cv::Mat &current, const cv::Mat &previous, const cv::Mat &mask);
void updateFPS();

// --- 마우스 콜백 함수 ---
void onMouse(int event, int x, int y, int flags, void *userdata)
{
    if (g_roiSelected)
        return;

    x = x * CAPTURE_WIDTH / 640;
    y = y * CAPTURE_HEIGHT / 480;

    if (event == cv::EVENT_LBUTTONDOWN)
    {
        g_points.push_back(cv::Point(x, y));
        g_drawing = true;
        std::cout << "점 추가: (" << x << ", " << y << ")" << std::endl;
    }
    else if (event == cv::EVENT_RBUTTONDOWN)
    {
        if (g_drawing && g_points.size() > 2)
        {
            g_drawing = false;
            g_roiSelected = true;

            g_roiMask = cv::Mat::zeros(cv::Size(CAPTURE_WIDTH, CAPTURE_HEIGHT), CV_8UC1);
            const cv::Point *pts[1] = {g_points.data()};
            int npts[] = {(int)g_points.size()};
            cv::fillPoly(g_roiMask, pts, npts, 1, cv::Scalar(255));

            std::cout << "\n=== ROI 선택 완료 ===" << std::endl;
            std::cout << "기준 프레임 캡처: 'b'" << std::endl;
            std::cout << "감지 제어: 'd_on' / 'd_off'" << std::endl;
            std::cout << "임계값 조절: 숫자 입력 또는 [/]" << std::endl;
            std::cout << "서보 설정: 'c'" << std::endl;
            std::cout << "통계: 's', 종료: 'q'" << std::endl;
            std::cout << "========================\n"
                      << std::endl;

            // Detection Controller 활성화
            g_detectionController->enable();
            g_detectionController->arm();
        }
    }
}

// --- 병 감지 액션 ---
// void pushBottle() {
//     auto now = std::chrono::high_resolution_clock::now();
//     auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

//     std::cout << "\n[" << timestamp << "] *** 병 감지! ***" << std::endl;
//     std::cout << "SAD: " << std::fixed << std::setprecision(2) << g_currentSAD
//               << " (임계값: " << g_detectionController->getThreshold() << ")" << std::endl;
// }

// --- 터미널 입력 처리 ---
void inputHandler()
{
    std::string input;
    while (!g_shouldExit)
    {
        if (!std::getline(std::cin, input))
            break;

        if (input.empty())
            continue;

        // Detection Controller 명령 처리
        if (g_detectionController->processCommand(input))
        {
            continue;
        }

        try
        {
            // 숫자 입력 시 임계값 직접 설정
            double newThreshold = std::stod(input);
            if (newThreshold > 0)
            {
                g_detectionController->setThreshold(newThreshold);
                std::cout << "임계값 설정: " << newThreshold << std::endl;
            }
        }
        catch (std::invalid_argument &)
        {
            // 기타 명령어 처리
            if (input == "]")
            {
                double current = g_detectionController->getThreshold();
                g_detectionController->setThreshold(current * 1.1);
                std::cout << "임계값 증가: " << g_detectionController->getThreshold() << std::endl;
            }
            else if (input == "[")
            {
                double current = g_detectionController->getThreshold();
                g_detectionController->setThreshold(current * 0.9);
                std::cout << "임계값 감소: " << g_detectionController->getThreshold() << std::endl;
            }
            else if (input == "e")
            {
                g_servoEnabled = !g_servoEnabled;
                std::cout << "서보 모터: " << (g_servoEnabled ? "활성화" : "비활성화") << std::endl;
            }
            else if (input == "c")
            {
                std::cout << "\n=== 서보 모터 설정 ===" << std::endl;
                std::cout << "밀기 각도 (0-180): ";
                int pushAngle;
                std::cin >> pushAngle;
                std::cout << "대기 각도 (0-180): ";
                int restAngle;
                std::cin >> restAngle;
                std::cout << "동작 시간 (100-5000 ms): ";
                int duration;
                std::cin >> duration;
                std::cin.ignore();

                configureServo(pushAngle, restAngle, duration);
            }
            else if (input == "v")
            {
                std::cout << "\n"
                          << readServoStatus() << std::endl;
            }
            else if (input == "s")
            {
                auto stats = g_detectionController->getStats();
                std::cout << "\n=== 성능 및 통계 ===" << std::endl;
                std::cout << "FPS: " << std::fixed << std::setprecision(1) << g_fps << std::endl;
                std::cout << "현재 SAD: " << g_currentSAD << std::endl;
                std::cout << "평균 SAD: " << stats.averageSAD << std::endl;
                std::cout << "최대 SAD: " << stats.maxSAD << std::endl;
                std::cout << "최소 SAD: " << stats.minSAD << std::endl;
                std::cout << "임계값: " << g_detectionController->getThreshold() << std::endl;
                std::cout << "감지 상태: " << g_detectionController->getStateString() << std::endl;
                std::cout << "총 감지 횟수: " << stats.totalDetections << std::endl;
                std::cout << "서보 동작 횟수: " << stats.servoActions << std::endl;
                std::cout << "서보 상태: " << (g_servoEnabled ? "활성화" : "비활성화") << std::endl;
                std::cout << "===================\n"
                          << std::endl;
            }
            else if (input == "q")
            {
                g_shouldExit = true;
                break;
            }
        }
    }
}

// --- 기준 프레임 캡처 ---
void captureBaseline()
{
    if (!g_currentFrame.empty() && !g_roiMask.empty())
    {
        g_baselineFrame = g_currentFrame.clone();
        std::cout << "기준 프레임 캡처 완료 (현재 상태를 기준으로 설정)" << std::endl;
    }
}

// --- 최적화된 SAD 계산 ---
double calculateFastSAD(const cv::Mat &current, const cv::Mat &previous, const cv::Mat &mask)
{
    if (current.empty() || previous.empty() || mask.empty())
    {
        return 0.0;
    }

    if (current.size() != previous.size() || current.size() != mask.size())
    {
        return 0.0;
    }

    cv::Mat diff;
    cv::absdiff(current, previous, diff);

    cv::Mat maskedDiff;
    cv::bitwise_and(diff, mask, maskedDiff);

    return cv::sum(maskedDiff)[0];
}

// --- FPS 업데이트 ---
void updateFPS()
{
    g_frameCounter++;
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - g_lastTime).count();

    if (duration >= 1000)
    {
        g_fps = (g_frameCounter * 1000.0) / duration;
        g_frameCounter = 0;
        g_lastTime = now;
    }
}

// --- 메인 함수 ---
int main()
{
    // 시그널 핸들러 설정
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    std::cout << "=== 고속 플라스틱병 감지 시스템 v4.0 (Detection Controller) ===" << std::endl;
    std::cout << "해상도: " << CAPTURE_WIDTH << "x" << CAPTURE_HEIGHT
              << " @ " << CAPTURE_FPS << "FPS" << std::endl;

    // Detection Controller 초기화
    DetectionConfig config;
    config.defaultThreshold = 500000.0; // 기본 임계값 500000
    config.cooldownMs = 1000;
    config.enableLogging = true;
    config.logFilePath = "/tmp/bottle_detection.log";

    g_detectionController = std::make_unique<DetectionController>(config);
    g_detectionController->setEventHandler(detectionEventHandler);

    // ================== TCP 서버 설정 시작 ==================
    g_tcpServer.setOnMessageReceived([&](const std::string &message)
                                     {
        std::cout << "[TCP Received] " << message << std::endl;
        
        // 수신된 명령어를 Detection Controller로 전달
        // 기존 터미널 입력 처리와 동일한 로직을 사용합니다.
        if (g_detectionController->processCommand(message)) {
            // команда 처리 성공
        } else {
            // 숫자나 다른 명령어가 들어올 경우의 처리 (필요 시 추가)
            std::cout << "[TCP] Unknown command: " << message << std::endl;
        } });

    int tcp_port = 8080; // 사용할 포트 번호
    if (!g_tcpServer.start(tcp_port))
    {
        std::cerr << "치명적 오류: TCP 서버를 시작할 수 없습니다. 포트 " << tcp_port << "가 사용 중인지 확인하세요." << std::endl;
    }
    else
    {
        std::cout << "TCP 서버가 포트 " << tcp_port << "에서 시작되었습니다." << std::endl;
    }
    // ================== TCP 서버 설정 종료 ==================

    // 서보 디바이스 연결
    if (!openServoDevice())
    {
        std::cout << "경고: 서보 디바이스 연결 실패. 감지만 수행합니다." << std::endl;
        g_servoEnabled = false;
    }
    else
    {
        // 기본 서보 설정
        configureServo(180, 0, 300);
    }

    // 카메라 파이프라인 설정
    std::string pipeline =
        "libcamerasrc ! "
        "video/x-raw,width=" +
        std::to_string(CAPTURE_WIDTH) +
        ",height=" + std::to_string(CAPTURE_HEIGHT) +
        ",framerate=" + std::to_string(CAPTURE_FPS) + "/1 ! "
                                                      "videoconvert ! "
                                                      "videoscale ! "
                                                      "appsink drop=true max-buffers=1";

    g_cap = new cv::VideoCapture(pipeline, cv::CAP_GSTREAMER);

    if (!g_cap->isOpened())
    {
        std::cerr << "오류: 카메라를 열 수 없습니다." << std::endl;
        closeServoDevice();
        delete g_cap;
        return -1;
    }

    g_cap->set(cv::CAP_PROP_BUFFERSIZE, 1);

    std::cout << "카메라 준비 완료!\n"
              << std::endl;
    std::cout << "=== 사용 방법 ===" << std::endl;
    std::cout << "1. 마우스 왼쪽 클릭: ROI 점 추가" << std::endl;
    std::cout << "2. 마우스 오른쪽 클릭: ROI 완성" << std::endl;
    std::cout << "3. 'b': 기준 프레임 캡처" << std::endl;
    std::cout << "4. 'd_on' / 'd_off': 감지 시 서보 동작 ON/OFF" << std::endl;
    std::cout << "5. '[' / ']': 임계값 10% 감소/증가" << std::endl;
    std::cout << "6. 숫자 입력: 임계값 직접 설정" << std::endl;
    std::cout << "7. 'e': 서보 모터 전체 ON/OFF" << std::endl;
    std::cout << "8. 'c': 서보 모터 설정" << std::endl;
    std::cout << "9. 'v': 서보 상태 확인" << std::endl;
    std::cout << "10. 's': 통계 보기" << std::endl;
    std::cout << "11. 'q': 종료" << std::endl;
    std::cout << "==================\n"
              << std::endl;

    cv::namedWindow("Camera Feed", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("Camera Feed", onMouse, NULL);

    // 입력 스레드 시작
    std::thread inputThread(inputHandler);

    // 프로세싱용 변수
    cv::Mat frame, grayFrame, blurredFrame, displayFrame;
    int errorCount = 0;
    const int MAX_ERRORS = 10;

    while (!g_shouldExit)
    {
        if (!g_cap->read(frame))
        {
            errorCount++;
            if (errorCount > MAX_ERRORS)
            {
                std::cerr << "프레임 읽기 실패가 계속됩니다. 종료합니다." << std::endl;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        errorCount = 0;

        if (frame.empty())
            continue;

        // 그레이스케일 변환
        if (frame.channels() > 1)
        {
            cv::cvtColor(frame, grayFrame, cv::COLOR_BGR2GRAY);
        }
        else
        {
            grayFrame = frame;
        }

        // 가우시안 블러 적용
        cv::GaussianBlur(grayFrame, blurredFrame, cv::Size(BLUR_SIZE, BLUR_SIZE), 0);

        // 현재 프레임 저장
        g_currentFrame = blurredFrame.clone();

        // 디스플레이용 프레임 준비 (확대)
        cv::resize(blurredFrame, displayFrame, cv::Size(640, 480), 0, 0, cv::INTER_LINEAR);

        if (!g_roiSelected)
        {
            // ROI 선택 모드
            if (g_drawing && g_points.size() > 0)
            {
                std::vector<cv::Point> displayPoints;
                for (const auto &pt : g_points)
                {
                    displayPoints.push_back(cv::Point(pt.x * 640 / CAPTURE_WIDTH,
                                                      pt.y * 480 / CAPTURE_HEIGHT));
                }

                for (size_t i = 0; i < displayPoints.size() - 1; ++i)
                {
                    cv::line(displayFrame, displayPoints[i], displayPoints[i + 1], cv::Scalar(255), 2);
                }
            }
            cv::putText(displayFrame, "Click points, right-click to close ROI",
                        cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255), 2);
        }
        else
        {
            // 감지 모드
            // ROI 영역 표시
            if (g_points.size() > 1)
            {
                std::vector<cv::Point> displayPoints;
                for (const auto &pt : g_points)
                {
                    displayPoints.push_back(cv::Point(pt.x * 640 / CAPTURE_WIDTH,
                                                      pt.y * 480 / CAPTURE_HEIGHT));
                }
                const cv::Point *pts[1] = {displayPoints.data()};
                int npts[] = {(int)displayPoints.size()};
                cv::polylines(displayFrame, pts, npts, 1, true, cv::Scalar(255), 2);
            }

            // 초기 프레임 설정
            if (g_prevFrame.empty())
            {
                g_prevFrame = blurredFrame.clone();
                g_baselineFrame = blurredFrame.clone();
                std::cout << "초기화 완료 - 감지 시작" << std::endl;
            }
            else
            {
                // SAD 계산
                if (!g_baselineFrame.empty())
                {
                    g_currentSAD = calculateFastSAD(blurredFrame, g_baselineFrame, g_roiMask);
                }
                else
                {
                    g_currentSAD = calculateFastSAD(blurredFrame, g_prevFrame, g_roiMask);
                }

                // 히스토리 업데이트
                g_sadHistory.push_back(g_currentSAD);
                if (g_sadHistory.size() > HISTORY_SIZE)
                {
                    g_sadHistory.pop_front();
                }

                // Detection Controller를 통한 병 감지
                bool shouldTrigger = g_detectionController->shouldTriggerServo(g_currentSAD);

                // DetectionController가 모든 감지, 상태 변경, 쿨다운, 재시동을 처리합니다.
                g_detectionController->shouldTriggerServo(g_currentSAD);

                // if (!g_isBottlePresent && g_currentSAD > threshold) {
                //     g_isBottlePresent = true;
                //     g_framesSinceDetection = 0;
                //     pushBottle();
                // } else if (g_isBottlePresent) {
                //     g_framesSinceDetection++;
                //     if (g_framesSinceDetection > DEBOUNCE_FRAMES && g_currentSAD < threshold * 0.6) {
                //         g_isBottlePresent = false;
                //         g_detectionController->reportPass();
                //         std::cout << "병 통과 완료" << std::endl;
                //     }
                // }

                // 정보 표시
                double threshold = g_detectionController->getThreshold();
                std::stringstream ss;
                ss << "FPS: " << std::fixed << std::setprecision(1) << g_fps
                   << " | SAD: " << std::setprecision(0) << g_currentSAD
                   << " / " << threshold;
                ss << " | " << g_detectionController->getStateString();

                cv::putText(displayFrame, ss.str(), cv::Point(10, 30),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255), 2);

                // 키 안내
                cv::putText(displayFrame, "d_on/d_off: detection | [ ] : threshold | s : stats",
                            cv::Point(10, 460),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200), 1);

                // SAD 레벨 바
                int barWidth = static_cast<int>((g_currentSAD / threshold) * 200);
                barWidth = std::min(barWidth, 200);
                cv::rectangle(displayFrame, cv::Point(10, 45),
                              cv::Point(10 + barWidth, 55), cv::Scalar(200), cv::FILLED);
                cv::rectangle(displayFrame, cv::Point(10, 45),
                              cv::Point(210, 55), cv::Scalar(255), 1);

                // if (g_isBottlePresent)
                // {
                //     cv::putText(displayFrame, "DETECTED!", cv::Point(10, 85),
                //                 cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(255), 3);
                //     cv::rectangle(displayFrame, cv::Point(5, 5),
                //                   cv::Point(635, 475), cv::Scalar(255), 5);
                // }

                // 감지 시 시각적 피드백
                // 'g_isBottlePresent' 대신 Controller의 현재 상태를 사용해 화면에 표시합니다.
                DetectionState currentState = g_detectionController->getState();
                if (currentState == DetectionState::TRIGGERED || currentState == DetectionState::COOLDOWN)
                {
                    cv::putText(displayFrame, "DETECTED!", cv::Point(10, 85),
                                cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(255), 3);
                    cv::rectangle(displayFrame, cv::Point(5, 5),
                                  cv::Point(635, 475), cv::Scalar(255), 5);
                }

                g_prevFrame = blurredFrame.clone();
            }
        }

        // FPS 업데이트
        updateFPS();

        // 그레이스케일을 3채널로 변환하여 컬러 표시
        cv::Mat displayColor;
        cv::cvtColor(displayFrame, displayColor, cv::COLOR_GRAY2BGR);
        cv::imshow("Camera Feed", displayColor);

        char key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 27)
        {
            g_shouldExit = true;
            break;
        }
        else if (key == 'b' && g_roiSelected)
        {
            captureBaseline();
        }
        else if (key == '[')
        {
            double current = g_detectionController->getThreshold();
            g_detectionController->setThreshold(current * 0.9);
            std::cout << "임계값 감소: " << g_detectionController->getThreshold() << std::endl;
        }
        else if (key == ']')
        {
            double current = g_detectionController->getThreshold();
            g_detectionController->setThreshold(current * 1.1);
            std::cout << "임계값 증가: " << g_detectionController->getThreshold() << std::endl;
        }
        else if (key == 'e')
        {
            g_servoEnabled = !g_servoEnabled;
            std::cout << "서보 모터: " << (g_servoEnabled ? "활성화" : "비활성화") << std::endl;
        }
        else if (key == 'v')
        {
            std::cout << "\n"
                      << readServoStatus() << std::endl;
        }
    }

    // 정리
    g_shouldExit = true;

    // 최종 통계 출력
    auto stats = g_detectionController->getStats();
    std::cout << "\n=== 최종 통계 ===" << std::endl;
    std::cout << "총 감지 횟수: " << stats.totalDetections << std::endl;
    std::cout << "서보 동작 횟수: " << stats.servoActions << std::endl;
    std::cout << "==================" << std::endl;

    // TCP 서버 중지
    g_tcpServer.stop();
    std::cout << "TCP 서버가 중지되었습니다." << std::endl;

    // 서보 디바이스 닫기
    closeServoDevice();

    // 카메라 해제
    if (g_cap)
    {
        g_cap->release();
        delete g_cap;
    }

    // 스레드 종료 대기
    if (inputThread.joinable())
    {
        std::cout << "\n입력 스레드 종료 중..." << std::endl;
        inputThread.join();
    }

    cv::destroyAllWindows();

    std::cout << "\n프로그램이 안전하게 종료되었습니다." << std::endl;
    return 0;
}
