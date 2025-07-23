#include "tcp_client.h"
#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>

TCPClient::TCPClient() : client_socket_(-1), is_connected_(false) {}

TCPClient::~TCPClient() {
    disconnect();
}

bool TCPClient::connectToServer(const std::string& ip, int port) {
    if (is_connected_) {
        std::cerr << "Already connected." << std::endl;
        return false;
    }

    client_socket_ = socket(AF_INET, SOCK_STREAM, 0);
    if (client_socket_ < 0) {
        std::cerr << "Error creating socket: " << strerror(errno) << std::endl;
        return false;
    }

    sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);

    if (inet_pton(AF_INET, ip.c_str(), &server_addr.sin_addr) <= 0) {
        std::cerr << "Invalid server IP address." << std::endl;
        close(client_socket_);
        return false;
    }

    if (connect(client_socket_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "Error connecting to server: " << strerror(errno) << std::endl;
        close(client_socket_);
        return false;
    }

    is_connected_ = true;
    return true;
}

void TCPClient::disconnect() {
    if (is_connected_) {
        close(client_socket_);
        is_connected_ = false;
        client_socket_ = -1;
        std::cout << "Disconnected from server." << std::endl;
    }
}

bool TCPClient::sendCommand(const std::string& command) {
    if (!is_connected_) {
        std::cerr << "Not connected to any server." << std::endl;
        return false;
    }

    std::string message = command + "\n"; // 서버에서 쉽게 파싱하도록 개행 문자 추가
    ssize_t written = write(client_socket_, message.c_str(), message.length());
    if (written < 0) {
        std::cerr << "Error sending command: " << strerror(errno) << std::endl;
        disconnect(); // 에러 발생 시 연결 끊기
        return false;
    }

    return true;
}