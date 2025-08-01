# Detection Controller Makefile

# 컴파일러 설정
CXX := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -O2 -g
LDFLAGS := -pthread

# GStreamer 설정
GST_CFLAGS := $(shell pkg-config --cflags gstreamer-1.0 gstreamer-rtsp-server-1.0 gstreamer-app-1.0)
GST_LIBS := $(shell pkg-config --libs gstreamer-1.0 gstreamer-rtsp-server-1.0 gstreamer-app-1.0)

# OpenCV 설정
OPENCV_FLAGS := $(shell pkg-config --cflags opencv4)
OPENCV_LIBS := $(shell pkg-config --libs opencv4)

# 소스 파일
CONTROLLER_SRC := detection_controller.cpp
MAIN_SRC := detect_ROI_v2.cpp
TCP_SERVER_SRC := tcp_server.cpp   # <--- TCP 서버 소스 추가

CONTROLLER_OBJ := detection_controller.o
MAIN_OBJ := detect_ROI_v2.o
TCP_SERVER_OBJ := tcp_server.o   # <--- TCP 서버 오브젝트 추가

# 클라이언트 소스 (별도 컴파일용)
CLIENT_SRC := tcp_client.cpp main_client.cpp
CLIENT_OBJ := tcp_client.o main_client.o

# 실행 파일
TARGET := detect_roi
CLIENT_TARGET := tcp_client

# 색상 정의
RED := \033[0;31m
GREEN := \033[0;32m
YELLOW := \033[0;33m
BLUE := \033[0;34m
NC := \033[0m

.PHONY: all clean controller main run help client

# 기본 타겟
all: $(TARGET)
	@echo "$(GREEN)Build complete! Run with: ./$(TARGET)$(NC)"

# 감지 제어 모듈 컴파일
$(CONTROLLER_OBJ): $(CONTROLLER_SRC) detection_controller.h
	@echo "$(BLUE)Compiling detection controller...$(NC)"
	$(CXX) $(CXXFLAGS) -c $(CONTROLLER_SRC) -o $(CONTROLLER_OBJ)

# TCP 서버 모듈 컴파일   <--- TCP 서버 컴파일 규칙 추가
$(TCP_SERVER_OBJ): $(TCP_SERVER_SRC) tcp_server.h
	@echo "$(BLUE)Compiling TCP server...$(NC)"
	$(CXX) $(CXXFLAGS) -c $(TCP_SERVER_SRC) -o $(TCP_SERVER_OBJ)

# 메인 프로그램 컴파일
$(MAIN_OBJ): $(MAIN_SRC) detection_controller.h tcp_server.h
	@echo "$(BLUE)Compiling main program...$(NC)"
	$(CXX) $(CXXFLAGS) $(OPENCV_FLAGS) $(GST_CFLAGS) -c $(MAIN_SRC) -o $(MAIN_OBJ)

# 링킹
$(TARGET): $(CONTROLLER_OBJ) $(MAIN_OBJ) $(TCP_SERVER_OBJ) # <--- 오브젝트 목록에 TCP 서버 추가
	@echo "$(BLUE)Linking...$(NC)"
	$(CXX) $(CONTROLLER_OBJ) $(MAIN_OBJ) $(TCP_SERVER_OBJ) -o $(TARGET) $(LDFLAGS) $(OPENCV_LIBS) $(GST_LIBS)

# 클라이언트 컴파일 타겟   <--- 클라이언트 컴파일 규칙 추가
client: $(CLIENT_SRC) tcp_client.h
	@echo "$(BLUE)Compiling TCP client...$(NC)"
	$(CXX) $(CXXFLAGS) $(CLIENT_SRC) -o $(CLIENT_TARGET)
	@echo "$(GREEN)Client build complete! Run with: ./$(CLIENT_TARGET) <ip> <port>$(NC)"

# 청소
clean:
	@echo "$(YELLOW)Cleaning build files...$(NC)"
	rm -f $(CONTROLLER_OBJ) $(MAIN_OBJ) $(TCP_SERVER_OBJ) $(TARGET) $(CLIENT_TARGET) $(CLIENT_OBJ)
	@echo "$(GREEN)Clean complete!$(NC)"

# 실행
run: $(TARGET)
	@echo "$(BLUE)Starting detection system...$(NC)"
	./$(TARGET)

# 도움말 (생략)
# ...