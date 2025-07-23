#include "tcp_server.h"
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <string.h>
#include <algorithm>

TCPServer::TCPServer() : server_socket_(-1), port_(0), is_running_(false) {}

TCPServer::~TCPServer() {
    stop();
}

bool TCPServer::start(int port) {
    if (is_running_) {
        std::cerr << "Server is already running." << std::endl;
        return false;
    }

    port_ = port;
    server_socket_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_socket_ < 0) {
        std::cerr << "Error creating socket: " << strerror(errno) << std::endl;
        return false;
    }

    // SO_REUSEADDR 옵션 설정 (서버 즉시 재시작 가능)
    int opt = 1;
    if (setsockopt(server_socket_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        std::cerr << "Error setting socket options: " << strerror(errno) << std::endl;
        close(server_socket_);
        return false;
    }

    sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(port_);

    if (bind(server_socket_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "Error binding socket: " << strerror(errno) << std::endl;
        close(server_socket_);
        return false;
    }

    if (listen(server_socket_, 5) < 0) {
        std::cerr << "Error listening on socket: " << strerror(errno) << std::endl;
        close(server_socket_);
        return false;
    }

    is_running_ = true;
    accept_thread_ = std::thread(&TCPServer::acceptLoop, this);

    return true;
}

void TCPServer::stop() {
    if (!is_running_) {
        return;
    }

    is_running_ = false;
    
    // 서버 소켓을 닫아 accept() 대기를 중단시킵니다.
    if (server_socket_ != -1) {
        shutdown(server_socket_, SHUT_RDWR);
        close(server_socket_);
        server_socket_ = -1;
    }
    
    if (accept_thread_.joinable()) {
        accept_thread_.join();
    }

    // 모든 클라이언트 스레드가 종료될 때까지 기다립니다.
    for (auto& t : client_threads_) {
        if (t.joinable()) {
            t.join();
        }
    }
    client_threads_.clear();
}

void TCPServer::setOnMessageReceived(std::function<void(const std::string&)> callback) {
    on_message_received_ = callback;
}

void TCPServer::acceptLoop() {
    while (is_running_) {
        sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        int client_socket = accept(server_socket_, (struct sockaddr*)&client_addr, &client_len);

        if (client_socket < 0) {
            if (is_running_) {
                std::cerr << "Error accepting client: " << strerror(errno) << std::endl;
            }
            continue;
        }

        std::cout << "[TCP] Client connected." << std::endl;
        client_threads_.emplace_back(&TCPServer::handleClient, this, client_socket);
    }
}

void TCPServer::handleClient(int client_socket) {
    char buffer[1024];
    while (is_running_) {
        memset(buffer, 0, sizeof(buffer));
        ssize_t bytes_read = read(client_socket, buffer, sizeof(buffer) - 1);

        if (bytes_read <= 0) {
            if (bytes_read == 0) {
                std::cout << "[TCP] Client disconnected." << std::endl;
            } else {
                std::cerr << "[TCP] Error reading from client: " << strerror(errno) << std::endl;
            }
            break;
        }

        // 수신된 데이터에서 개행 문자 제거
        std::string message(buffer, bytes_read);
        message.erase(std::remove(message.begin(), message.end(), '\n'), message.end());
        message.erase(std::remove(message.begin(), message.end(), '\r'), message.end());

        if (on_message_received_) {
            on_message_received_(message);
        }
    }
    close(client_socket);
}