#include "tcp_client.h"
#include <iostream>

void print_usage() {
    std::cout << "Usage: ./tcp_client <server_ip> <port>" << std::endl;
    std::cout << "Example: ./tcp_client 192.168.0.10 8080" << std::endl;
}

int main(int argc, char* argv[]) {
    if (argc != 3) {
        print_usage();
        return 1;
    }

    std::string server_ip = argv[1];
    int port = std::stoi(argv[2]);

    TCPClient client;
    if (!client.connectToServer(server_ip, port)) {
        return 1;
    }

    std::cout << "Successfully connected to server at " << server_ip << ":" << port << std::endl;
    std::cout << "Enter commands ('d_on', 'd_off', etc.) or 'exit' to quit." << std::endl;

    std::string command;
    while (true) {
        std::cout << "> ";
        std::getline(std::cin, command);

        if (command == "exit") {
            break;
        }

        if (!client.sendCommand(command)) {
            std::cerr << "Failed to send command. The connection may have been lost." << std::endl;
            break;
        }
    }

    client.disconnect();
    return 0;
}