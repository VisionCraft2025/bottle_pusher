#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#include <string>
#include <functional>
#include <thread>
#include <atomic>
#include <vector>

class TCPServer {
public:
    TCPServer();
    ~TCPServer();

    // 서버를 지정된 포트에서 시작합니다 (백그라운드 스레드에서 실행).
    bool start(int port);

    // 서버를 중지합니다.
    void stop();

    // 클라이언트로부터 메시지(명령어)를 수신했을 때 호출될 콜백 함수를 설정합니다.
    void setOnMessageReceived(std::function<void(const std::string&)> callback);

private:
    void acceptLoop(); // 클라이언트 연결을 수락하는 메인 루프입니다.
    void handleClient(int client_socket); // 개별 클라이언트와의 통신을 처리합니다.

    int server_socket_;
    int port_;
    std::atomic<bool> is_running_;
    std::thread accept_thread_;
    std::vector<std::thread> client_threads_;
    std::function<void(const std::string&)> on_message_received_;
};

#endif // TCP_SERVER_H