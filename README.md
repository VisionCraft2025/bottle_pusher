# 고속 플라스틱병 감지 시스템

-----

## 개요

이 프로젝트는 **라즈베리파이** 기반의 고속 컨베이어 벨트에서 플라스틱병을 실시간으로 감지하고 서보 모터를 이용해 제거하는 시스템입니다. **SAD (Sum of Absolute Differences)** 알고리즘을 사용하여 빠르게 움직이는 물체를 감지하며, **RTSP 스트리밍**과 **TCP 원격 제어**를 지원합니다.

### 주요 기능

  * **실시간 객체 감지**: SAD 알고리즘을 사용한 고속 움직임 감지.
  * **서보 모터 제어**: 감지된 병을 밀어내는 서보 모터 제어 기능.
  * **RTSP 스트리밍**: `rtsp://IP:8554/process3`를 통해 실시간 영상을 네트워크로 스트리밍.
  * **TCP 원격 제어**: 네트워크를 통한 시스템 원격 제어.
  * **Detection Controller**: 상태 기반의 감지 시스템 관리.
  * **헤드리스 모드**: GUI 없이 터미널에서 전체 제어 가능.

-----

## 시스템 요구사항

  * **하드웨어**:
      * Raspberry Pi 4 (권장)
      * Raspberry Pi Camera Module
      * 서보 모터 (커스텀 드라이버 필요)
  * **소프트웨어**:
      * OpenCV 4.x
      * GStreamer 1.0
      * RTSP Server 라이브러리

-----

## 설치 방법

### 1\. 의존성 설치

터미널에서 다음 명령어를 순서대로 실행하여 필요한 라이브러리를 설치합니다.

```bash
sudo apt-get update
sudo apt-get install libopencv-dev
sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
sudo apt-get install libgstrtspserver-1.0-dev
sudo apt-get install gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-bad
```

### 2\. 컴파일

`detect_src` 디렉터리로 이동하여 소스 코드를 컴파일합니다.

```bash
cd detect_src
make clean
make -j4
```

### 3\. 서보 드라이버 설치 (필요 시)

`servo_driver` 디렉터리로 이동하여 커스텀 서보 드라이버를 설치합니다.

```bash
cd servo_driver
sudo make load
```

-----

## 사용 방법

### 1\. 프로그램 실행

컴파일 후 생성된 실행 파일을 실행합니다.

```bash
./detect_roi
```

### 2\. ROI (Region of Interest) 설정

프로그램 시작 시 ROI 설정 여부를 묻는 메시지가 나타납니다.

```
ROI를 설정하시겠습니까? (y/n): y
```

ROI 좌표를 입력하고 `done`을 입력하여 설정을 완료합니다. 좌표 범위는 `x(0-319)`, `y(0-239)`입니다.

### 3\. 주요 명령어

프로그램 실행 중 다음과 같은 단일 문자 또는 숫자 명령어를 입력하여 시스템을 제어할 수 있습니다.

  * `r`: ROI 재설정
  * `b`: 기준 프레임 캡처 (컨베이어가 비어있을 때)
  * `d_on`: 감지 시 서보 동작 활성화
  * `d_off`: 감지 시 서보 동작 비활성화
  * `[` / `]`: 임계값 10% 감소/증가
  * 숫자 입력 (예: `500000`): 임계값 직접 설정
  * `e`: 서보 모터 전체 ON/OFF
  * `c`: 서보 모터 설정 (각도, 동작 시간)
  * `v`: 서보 상태 확인
  * `s`: 통계 보기
  * `q`: 프로그램 종료

### 4\. RTSP 스트리밍 접속

**VLC**나 **ffplay**와 같은 플레이어를 사용하여 스트리밍 영상을 확인할 수 있습니다.

```bash
# VLC
vlc rtsp://라즈베리파이IP:8554/process3

# ffplay
ffplay rtsp://라즈베리파이IP:8554/process3
```

### 5\. TCP 원격 제어

별도의 클라이언트 프로그램을 사용하여 TCP를 통해 시스템을 원격으로 제어할 수 있습니다.

```bash
./tcp_client 192.168.0.64 8080
> d_on     # 감지 활성화
> d_off    # 감지 비활성화
> 1000000  # 임계값 설정
> exit     # 종료
```

-----

## Detection Controller 상태

시스템은 다음 5가지 상태로 동작하며, `detection_controller.cpp`에서 관리됩니다.

  * **DISABLED**: 비활성화 상태
  * **ENABLED**: 활성화 상태이지만 서보 동작은 없음
  * **ARMED**: 감지 및 서보 동작 준비 완료
  * **TRIGGERED**: 객체 감지 상태
  * **COOLDOWN**: 서보 동작 후 대기 상태 (자동으로 `ARMED`로 복귀)

-----

## 파일 구조

```
detect_src/
├── detect_ROI_v2.cpp       # 메인 프로그램
├── detection_controller.cpp/h   # 감지 상태 관리
├── tcp_server.cpp/h        # TCP 서버
├── tcp_client.cpp/h        # TCP 클라이언트
├── Makefile               # 빌드 설정 파일
└── coordinate.txt         # ROI 좌표 예시
```

-----

## 성능 최적화 설정

  * **캡처 해상도**: 320x240 @ 30FPS
  * **가우시안 블러 크기**: 5x5
  * **기본 SAD 임계값**: 500,000
  * **쿨다운 시간**: 1000ms

-----

## 문제 해결

### 카메라가 열리지 않을 때

다음 명령어를 통해 카메라 모듈 상태를 확인합니다.

```bash
vcgencmd get_camera
libcamera-hello
```

### RTSP 스트리밍이 안 될 때

GStreamer 디버그를 활성화하여 문제를 파악합니다.

```bash
export GST_DEBUG=3
./detect_roi
```

### 서보 모터가 동작하지 않을 때

서보 드라이버가 제대로 로드되었는지 확인합니다.

```bash
lsmod | grep servo
ls -la /dev/servo_pusher
```

-----

## 주의사항

  * **ROI**는 컨베이어 벨트 위의 실제 감지 영역에 맞춰 설정해야 합니다.
  * **기준 프레임**(`b` 명령어)은 컨베이어에 아무것도 없을 때 캡처해야 합니다.
  * **임계값**은 환경에 따라 최적의 값으로 조정이 필요합니다.
  * **서보 모터**의 각도와 동작 시간은 물리적 설치 상황에 맞게 조정해야 합니다.

-----

### 라이선스

이 프로젝트는 교육 및 연구 목적으로 제작되었습니다.