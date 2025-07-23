# Detection Controller Makefile

# 컴파일러 설정
CXX := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -O2 -g
LDFLAGS := -pthread

# OpenCV 설정
OPENCV_FLAGS := $(shell pkg-config --cflags opencv4)
OPENCV_LIBS := $(shell pkg-config --libs opencv4)

# 소스 파일
CONTROLLER_SRC := detection_controller.cpp
MAIN_SRC := detect_ROI_v2.cpp
CONTROLLER_OBJ := detection_controller.o
MAIN_OBJ := detect_ROI_v2.o

# 실행 파일
TARGET := detect_roi

# 색상 정의
RED := \033[0;31m
GREEN := \033[0;32m
YELLOW := \033[0;33m
BLUE := \033[0;34m
NC := \033[0m

.PHONY: all clean controller main run help

# 기본 타겟
all: $(TARGET)
	@echo "$(GREEN)Build complete! Run with: ./$(TARGET)$(NC)"

# 감지 제어 모듈 컴파일
$(CONTROLLER_OBJ): $(CONTROLLER_SRC) detection_controller.h
	@echo "$(BLUE)Compiling detection controller...$(NC)"
	$(CXX) $(CXXFLAGS) -c $(CONTROLLER_SRC) -o $(CONTROLLER_OBJ)

# 메인 프로그램 컴파일
$(MAIN_OBJ): $(MAIN_SRC) detection_controller.h
	@echo "$(BLUE)Compiling main program...$(NC)"
	$(CXX) $(CXXFLAGS) $(OPENCV_FLAGS) -c $(MAIN_SRC) -o $(MAIN_OBJ)

# 링킹
$(TARGET): $(CONTROLLER_OBJ) $(MAIN_OBJ)
	@echo "$(BLUE)Linking...$(NC)"
	$(CXX) $(CONTROLLER_OBJ) $(MAIN_OBJ) -o $(TARGET) $(LDFLAGS) $(OPENCV_LIBS)

# 청소
clean:
	@echo "$(YELLOW)Cleaning build files...$(NC)"
	rm -f $(CONTROLLER_OBJ) $(MAIN_OBJ) $(TARGET)
	@echo "$(GREEN)Clean complete!$(NC)"

# 실행
run: $(TARGET)
	@echo "$(BLUE)Starting detection system...$(NC)"
	./$(TARGET)

# 도움말
help:
	@echo "$(BLUE)Detection Controller Build System$(NC)"
	@echo "$(GREEN)Available targets:$(NC)"
	@echo "  $(YELLOW)make$(NC)         - Build all components"
	@echo "  $(YELLOW)make clean$(NC)   - Clean build files"
	@echo "  $(YELLOW)make run$(NC)     - Build and run the program"
	@echo "  $(YELLOW)make help$(NC)    - Show this help message"
	@echo ""
	@echo "$(BLUE)Dependencies:$(NC)"
	@echo "  - OpenCV 4.x"
	@echo "  - C++17 compatible compiler"
	@echo "  - pthread library"
	@echo ""
	@echo "$(BLUE)Detection Commands:$(NC)"
	@echo "  d_on     - Enable servo action on detection"
	@echo "  d_off    - Disable servo action on detection"
	@echo "  enable   - Enable detection system"
	@echo "  disable  - Disable detection system"