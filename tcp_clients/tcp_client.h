#ifndef TCP_CLIENT_H
#define TCP_CLIENT_H

#include <string>

class TCPClient {
public:
    TCPClient();
    ~TCPClient();

    // 서버에 연결합니다.
    bool connectToServer(const std::string& ip, int port);

    // 서버와의 연결을 해제합니다.
    void disconnect();

    // 서버에 명령어를 전송합니다.
    bool sendCommand(const std::string& command);

private:
    int client_socket_;
    bool is_connected_;
};

#endif // TCP_CLIENT_H