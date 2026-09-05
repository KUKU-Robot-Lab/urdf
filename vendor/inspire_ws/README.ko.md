# 정밀 핸드 제어 시스템 (Inspire / ROS2)

C++ 및 ROS2 기반의 다중 디바이스 정밀 핸드 (Dexterous Hand) 제어 시스템. 하위 계층에서 RS485 / CANFD 등을 통해 여러 Inspire 시리즈 정밀 핸드와 통신합니다. 노드 패키지 이름은 **`inspire_control_ros2`** 입니다.

## 프로젝트 소개

본 프로젝트는 모듈화된 정밀 핸드 제어 시스템으로, 다음 기능을 지원합니다:
- ✅ **다중 디바이스 지원**: 여러 정밀 핸드 디바이스(예: 왼손, 오른손)를 동시 제어
- ✅ **다중 프로토콜 지원**: 팩토리 패턴으로 여러 통신 프로토콜(RH56F1_485, RH5DG2_485 등) 지원
- ✅ **동적 설정**: YAML로 디바이스 프로토콜과 ROS2 토픽/서비스 설정
- ✅ **이중 통신 모드**: 토픽(실시간 제어)과 서비스(요청 시 호출) 모두 지원
- ✅ **비동기 시리얼 통신**: Boost.Asio 기반 비동기 시리얼 통신, 타임아웃 및 오류 처리 지원
- ✅ **통합 로그 시스템**: 전역 로그 매니저, 파일 회전 및 레벨 제어 지원

## 프로젝트 구조

```
serial_control/
├── src/                          # 소스 코드 디렉토리
│   ├── include/                  # 헤더 파일
│   │   ├── protocol.hpp          # 프로토콜 추상 기본 클래스
│   │   ├── RH56F1_485_protocol.hpp
│   │   ├── RH5DG2_485_protocol.hpp
│   │   ├── serial_port.hpp       # 시리얼 통신
│   │   ├── device_manager.hpp    # 디바이스 매니저
│   │   ├── config_loader.hpp     # 설정 로더
│   │   ├── logger_manager.hpp    # 로그 매니저
│   │   ├── ring_buffer.hpp       # 링 버퍼
│   │   └── protocol_factory.hpp  # 프로토콜 팩토리
│   ├── src/                      # 소스 파일
│   │   ├── protocol_factory.cpp
│   │   ├── RH56F1_485_protocol.cpp
│   │   ├── RH5DG2_485_protocol.cpp
│   │   ├── serial_port.cpp
│   │   ├── device_manager.cpp
│   │   ├── config_loader.cpp
│   │   ├── logger_manager.cpp
│   │   └── ring_buffer.cpp
│   ├── examples/                 # 예제 프로그램
│   │   └── main.cpp              # 다중 디바이스 병렬 제어 예제
│   ├── config/                   # 설정 파일
│   │   ├── device_protocol_config.yaml    # 디바이스 프로토콜 설정
│   │   └── RH56F1.yaml, RH5DG2.yaml    # 디바이스 설정 예제
│   └── ros2/                          # ROS2 워크스페이스 (colcon 최상위)
│       └── src/
│           ├── driver/                # 패키지 inspire_control_ros2
│           │   ├── src/               # 노드, RegisterController, 모델별 어댑터
│           │   ├── include/
│           │   ├── config/            # device_protocol_config.yaml, ros2_controller_config*.yaml
│           │   └── launch/            # inspire_control_*.launch.py
│           └── interfaces/
│               ├── RH5DG2/            # 인터페이스 패키지 rh5dg2_interfaces (13 자유도)
│               └── RH56F1/            # 인터페이스 패키지 rh56f1_interfaces (6 자유도)
├── architecture.ko.md            # 프로젝트 전체 아키텍처 문서
├── module_usage.ko.md            # 각 모듈 상세 사용 설명서
├── dependencies.ko.md            # 의존성 목록 및 설치 안내
├── install_dependencies.sh       # 의존성 설치 스크립트 (원클릭)
└── README.ko.md                  # 본 파일
```

### ROS2 인터페이스 설명 (리팩토링 이후)

| 패키지 이름 | 역할 |
|------------|------|
| **inspire_control_ros2** | 노드와 드라이버 로직: `inspire_control_node`, `RegisterController`, `RH5DG2InterfaceAdapter` / `RH56F1InterfaceAdapter`. 설정 파일은 `share/inspire_control_ros2/config`에 설치됩니다. |
| **rh5dg2_interfaces** | RH5DG2 (13 자유도) 전용 `msg`/`srv` (예: `SetAngle1`, `GetAngleAct1`, `Setforce`, `Geterror` 등). |
| **rh56f1_interfaces** | RH56 시리즈 (6 자유도) 전용 `msg`/`srv`. |

**`device_protocol_config.yaml`** 에서 **`protocol.type`** 설정 (예: **`RH5DG2_485`**, **`RH56F1_485`**, **`RH5DG2_canfd`** 등). 시작 시 **`interfaces_profile`** (`RH5DG2` / `RH56F1`)을 자동 추론하여 해당 어댑터를 생성하고, **`rh5dg2_interfaces` 또는 `rh56f1_interfaces`** 의 ROS 타입과 바인딩합니다.

빌드 시 워크스페이스의 인터페이스 패키지와 함께 빌드해야 합니다 ( [프로젝트 빌드](#3-프로젝트-빌드) 참조).

## 빠른 시작

> **💡 빠른 설치**: 자동 설치 스크립트로 모든 의존성을 한 번에 설치하는 것을 권장합니다
> ```bash
> ./install_dependencies.sh
> ```
> 자세한 내용은 [의존성 설치](#2-의존성-설치) 섹션 참조

### 1. 환경 요구 사항

- **OS**: Linux (Ubuntu 22.04+)
- **ROS2**: Humble 이상
- **C++ 표준**: C++17
- **컴파일러**: GCC 9+ 또는 Clang 10+
- **빌드 도구**: CMake 3.10+

### 2. 의존성 설치

#### 2.1 시스템 의존성

**Ubuntu/Debian**:

```bash
# 패키지 목록 업데이트
sudo apt update

# 기본 빌드 도구 설치
sudo apt install -y \
    build-essential \
    cmake \
    pkg-config \
    git \
    wget \
    curl

# C++ 컴파일러와 툴체인 설치
sudo apt install -y \
    gcc \
    g++ \
    make \
    libc6-dev
```

#### 2.2 ROS2 의존성

**ROS2 Humble 설치 (미설치 시)**:

```bash
# 로케일 설정
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# ROS2 소스 추가
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl gnupg lsb-release

# ROS2 GPG 키 추가
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# ROS2 Humble 설치
sudo apt update
sudo apt install -y ros-humble-desktop

# ROS2 개발 도구 설치
sudo apt install -y \
    ros-humble-rclcpp \
    ros-humble-std-msgs \
    ros-humble-std-srvs \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    python3-colcon-common-extensions \
    python3-rosdep

# rosdep 초기화
sudo rosdep init
rosdep update

# ROS2 환경 설정 (~/.bashrc 에 추가)
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### 2.3 서드파티 라이브러리 의존성

**Boost 설치**:

```bash
# Boost 개발 라이브러리 설치 (Boost.Asio 포함)
sudo apt install -y \
    libboost-system-dev \
    libboost-thread-dev \
    libboost-dev
```

**yaml-cpp 설치**:

```bash
sudo apt install -y libyaml-cpp-dev
```

**spdlog 설치**:

```bash
# 방법 1: apt 로 설치 (권장)
sudo apt install -y libspdlog-dev

# 방법 2: 소스에서 빌드 (apt 버전이 요구사항 미달인 경우)
cd /tmp
git clone https://github.com/gabime/spdlog.git
cd spdlog
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)
sudo make install
```

#### 2.4 시리얼 포트 권한 설정

**시리얼 포트 접근 권한 설정**:

```bash
# 방법 1: dialout 그룹에 사용자 추가 (권장, 영구 적용)
sudo usermod -a -G dialout $USER

# 방법 2: 임시 권한 부여 (재부팅 시마다 재설정 필요)
sudo chmod 666 /dev/ttyUSB0

# 참고: 방법 1 은 재로그인 후 적용
# 권한 확인
groups | grep dialout
```

**시리얼 디바이스 확인**:

```bash
# 시리얼 디바이스 목록
ls -l /dev/ttyUSB*

# 시리얼 정보 확인
dmesg | grep ttyUSB
```

#### 2.5 전체 의존성 목록

**시스템 의존성**:
- `build-essential` - 기본 빌드 도구
- `cmake` (>= 3.10) - 빌드 시스템
- `pkg-config` - 패키지 설정 도구
- `gcc` / `g++` (>= 9) - C++ 컴파일러
- `make` - 빌드 도구

**ROS2 의존성 (apt)**:
- `ros-humble-desktop` - ROS2 데스크톱 (또는 `ros-humble-rclcpp` 등 필요한 패키지 설치)
- `ros-humble-rclcpp` - ROS2 C++ 클라이언트 라이브러리
- `ros-humble-std-msgs` - ROS2 표준 메시지
- `ros-humble-rosidl-default-generators` - ROS2 인터페이스 생성기
- `ros-humble-rosidl-default-runtime` - ROS2 인터페이스 런타임
- `python3-colcon-common-extensions` - Colcon 빌드 도구 확장
- `python3-rosdep` - ROS 의존성 관리 도구 (선택)

**본 저장소의 ROS2 워크스페이스 패키지 (소스 빌드, apt 아님)**: `rh5dg2_interfaces`, `rh56f1_interfaces`, `inspire_control_ros2`. 위의 「ROS2 인터페이스 설명」과 `dependencies.ko.md` 참조.

**서드파티 라이브러리**:
- `libboost-system-dev` - Boost 시스템 라이브러리 (Boost.Asio 포함)
- `libboost-thread-dev` - Boost 스레드 라이브러리
- `libboost-dev` - Boost 개발 라이브러리
- `libyaml-cpp-dev` - yaml-cpp 개발 라이브러리
- `libspdlog-dev` - spdlog 개발 라이브러리

**원클릭 설치 스크립트**:

```bash
#!/bin/bash
# 전체 의존성 설치 스크립트

echo "=== 시스템 의존성 설치 ==="
sudo apt update
sudo apt install -y \
    build-essential \
    cmake \
    pkg-config \
    git \
    wget \
    curl \
    gcc \
    g++ \
    make \
    libc6-dev

echo "=== Boost 라이브러리 설치 ==="
sudo apt install -y \
    libboost-system-dev \
    libboost-thread-dev \
    libboost-dev

echo "=== yaml-cpp 라이브러리 설치 ==="
sudo apt install -y libyaml-cpp-dev

echo "=== spdlog 라이브러리 설치 ==="
sudo apt install -y libspdlog-dev

echo "=== 시리얼 포트 권한 설정 ==="
sudo usermod -a -G dialout $USER

echo "=== 의존성 설치 완료 ==="
echo "참고: 시리얼 포트 권한 설정은 재로그인 후 적용됩니다"
echo "실행: newgrp dialout 또는 재로그인"
```

#### 2.6 원클릭 설치 스크립트 (권장)

**자동 설치 스크립트 사용**:

```bash
cd /home/ubuntu/serial_control
chmod +x install_dependencies.sh
./install_dependencies.sh
```

스크립트는 자동으로:
- OS 감지
- 모든 시스템 의존성 설치
- Boost, yaml-cpp, spdlog 라이브러리 설치
- 시리얼 포트 권한 설정
- ROS2 설치 상태 확인
- 상세한 설치 피드백 제공

#### 2.7 설치 검증

**시스템 의존성 검증**:

```bash
# CMake 버전 확인
cmake --version  # >= 3.10 이어야 함

# GCC 버전 확인
gcc --version    # >= 9 이어야 함

# G++ 버전 확인
g++ --version    # >= 9 이어야 함
```

**ROS2 설치 검증**:

```bash
# ROS2 환경 확인
echo $ROS_DISTRO  # humble 표시되어야 함

# ROS2 패키지 확인
ros2 pkg list | grep rclcpp

# colcon 확인
colcon --version
```

**서드파티 라이브러리 검증**:

```bash
# Boost 확인
pkg-config --modversion boost

# yaml-cpp 확인
pkg-config --modversion yaml-cpp

# spdlog 확인 (apt 로 설치한 경우)
dpkg -l | grep spdlog
```

**시리얼 포트 권한 검증**:

```bash
# 사용자 그룹 확인
groups | grep dialout

# 시리얼 디바이스 확인
ls -l /dev/ttyUSB*  # 사용자에게 읽기/쓰기 권한 있어야 함
```

### 3. 프로젝트 빌드

#### 코어 라이브러리 빌드 (비-ROS2)

```bash
cd /home/ubuntu/serial_control/src
mkdir -p build && cd build
cmake ..
make
```

#### ROS2 워크스페이스 빌드 (인터페이스 패키지 + 노드 패키지)

```bash
cd /home/ubuntu/serial_control/src/ros2
source /opt/ros/humble/setup.bash   # 또는 설치된 ROS2 distro
colcon build --packages-select rh5dg2_interfaces rh56f1_interfaces inspire_control_ros2
source install/setup.bash
```

노드 코드만 변경했다면 `inspire_control_ros2` 만 빌드해도 되지만, **최초 클론 또는 인터페이스 패키지가 변경되었다면** 반드시 두 `*_interfaces` 패키지도 함께 빌드하세요.

### 4. 디바이스 설정

**`src/ros2/src/driver/config/device_protocol_config.yaml`** (또는 launch 와 일치하는 `--device-config` 경로) 편집:

```yaml
protocol:
  type: RH56F1_485

devices:
  - name: hand_left
    port: /dev/ttyUSB0
    baudrate: 115200
    Hand_ID: 1
```

### 5. 노드 실행

#### 단일 디바이스 모드

```bash
ros2 launch inspire_control_ros2 inspire_control_single_device.launch.py \
  device_name:=hand_left
```

#### 다중 디바이스 모드

```bash
ros2 launch inspire_control_ros2 inspire_control_multi_device.launch.py
```

### 6. 사용 예제

아래 예제는 **`protocol.type`** 이 RH5DG2 시리즈(**13** 관절)임을 가정합니다. **RH56F1** 시리즈의 경우 패키지명을 **`rh56f1_interfaces`** 로 변경하고 **`joint_values` 길이를 6** 으로 설정하세요. `ros2 interface show <패키지>/<타입>` 으로 필드를 확인할 수도 있습니다.

**`hand_id` 와 노드 바인딩**: 인바운드 토픽/서비스의 **`hand_id`** 는 **`device_protocol_config.yaml`** 에서 해당 디바이스의 **`Hand_ID`** 와 일치해야 하며, 그렇지 않으면 노드가 레지스터 쓰기를 거부(`accepted: false`)하거나 구독 콜백을 무시합니다. **`hand_id: 0`** 은 미지정으로 간주되어 노드가 수용합니다 (id 미지정 호환).

#### 제어 명령 발행 (토픽 모드)

```bash
# 각도 명령 (현장 캘리브레이션에 따라 값 조정)
ros2 topic pub --once /hand_left/angle_set rh5dg2_interfaces/msg/SetAngle1 \
  "{hand_id: 1, joint_values: [100,100,100,100,100,100,100,100,100,100,100,100,100]}"
```

#### 상태 데이터 구독 (토픽 모드)

```bash
ros2 topic echo /hand_left/angle_actual
```

#### 서비스 호출 (서비스 모드)

```bash
# 각도 설정 서비스 (angleSet 레지스터에 매핑)
ros2 service call /hand_left/set_angle rh5dg2_interfaces/srv/Setangle \
  "{command: '', hand_id: 1, joint_values: [100,100,100,100,100,100,100,100,100,100,100,100,100]}"

# 오류 코드 읽기 (예시)
ros2 service call /hand_left/get_errorCode rh5dg2_interfaces/srv/Geterror \
  "{query: '', hand_id: 1}"

# 디바이스 통신 ID 설정
ros2 service call /hand_left/set_id rh5dg2_interfaces/srv/Setid \
  "{hand_id: 1, device_id: 1}"
```

## 문서 설명

### 프로젝트 아키텍처

📖 **[architecture.ko.md](architecture.ko.md)**

포함 내용:
- 시스템 전체 아키텍처 다이어그램
- 모듈 간 관계와 데이터 흐름
- 스레드 모델
- 시작 흐름
- 확장 포인트

### 모듈 사용 설명

📖 **[module_usage.ko.md](module_usage.ko.md)**

포함 내용:
- 각 모듈 기능 요약
- 주요 클래스와 함수 설명
- 설정 파라미터 설명
- 사용 예제
- 데이터 흐름과 통신 방식

### 의존성 목록

📖 **[dependencies.ko.md](dependencies.ko.md)**

포함 내용:
- 전체 의존성 목록
- 버전 요구 사항
- 설치 명령
- 검증 방법
- 자주 묻는 질문

### 프로토콜 형식 설명

📖 **[src/document/RH56F1_485协议格式说明.md](src/document/RH56F1_485协议格式说明.md)**

포함 내용:
- 읽기/쓰기 요청 형식
- 읽기/쓰기 응답 형식
- 각 바이트의 의미
- 체크섬 계산
- 완전한 예제

## 핵심 모듈

### 1. 시리얼 통신 모듈 (SerialPortBase)

Boost.Asio 기반 비동기 시리얼 통신, 블로킹 읽기/쓰기와 타임아웃 메커니즘 지원.

**주요 기능**:
- 비동기 수신
- 블로킹 송신
- 타임아웃 읽기
- 스레드 안전

### 2. 프로토콜 추상 계층 (Protocol)

프로토콜 추상 기본 클래스, 통합된 프로토콜 인터페이스 정의. 여러 프로토콜 구현 지원 (RH56F1_485, RH5DG2_485 등).

**주요 기능**:
- 명령 빌드
- 응답 파싱
- 체크섬 검증
- 레지스터 읽기/쓰기

### 3. 디바이스 매니저 (DeviceManager)

여러 시리얼 디바이스를 관리하고, 포트와 디바이스 객체 매핑 유지.

**주요 기능**:
- 디바이스 추가/제거
- 디바이스 조회
- 다중 디바이스 관리

### 4. ROS2 컨트롤러 (RegisterController)

ROS2 디바이스 제어 노드. **`InterfaceAdapter`** 를 통해 **`rh5dg2_interfaces` / `rh56f1_interfaces`** 의 메시지 및 서비스 타입을 사용.

**주요 기능**:
- 토픽: 명령 구독, 상태 발행 (메시지 타입은 **`device_protocol_config.yaml`** 의 **`protocol.type`** 으로부터 추론)
- 서비스: 각 기능에 대응하는 독립 `.srv`, 통합 Register 서비스는 더 이상 사용하지 않음
- 타이머 루프: 기본 50Hz (`update_rate` 설정 가능)

### 5. 설정 시스템 (ConfigLoader)

YAML 파일에서 설정을 로드, 디바이스 설정과 로그 설정 지원.

**주요 기능**:
- 디바이스 설정 로드
- 프로토콜 객체 생성
- 로그 시스템 설정

### 6. 로그 시스템 (LoggerManager)

spdlog 기반 통합 로그 관리.

**주요 기능**:
- 콘솔 및 파일 출력
- 로그 회전
- 레벨 제어
- 스레드 안전

## 통신 방식

### 토픽 모드 (Topic)

**특징**:
- 실시간성 높음
- 연속 제어에 적합
- 타이머 루프로 읽기 및 발행

**사용 시나리오**:
- 실시간 각도 제어
- 실시간 힘 제어
- 상태 모니터링

### 서비스 모드 (Service)

**특징**:
- 요청 시 호출
- 타이머 루프에 참여하지 않음
- 단일 작업에 적합

**사용 시나리오**:
- 디바이스 설정 (ID, 보드레이트 등)
- 오류 조회
- 상태 조회

## 설정 파일

### 디바이스 프로토콜 설정 (device_protocol_config.yaml)

```yaml
protocol:
  type: RH56F1_485

devices:
  - name: hand_left
    port: /dev/ttyUSB0
    baudrate: 115200
    Hand_ID: 1

logging:
  level: DEBUG
  file: logs/hand_control.log
  console: true
  file_enable: true
  max_file_size_mb: 10
  max_files: 5
```

### ROS2 컨트롤러 설정 (ros2_controller_config.yaml)

```yaml
device_nodes:
  - device: hand_left
    update_rate: 50
    publish_header:
      frame_id: "hand_left"
    joint_names:
      - "hand_left/joint_0"
      # ... 총 13 개 (RH5DG2) 또는 6 개 (RH56F1)

    topics:
      - name: angle_control
        registers:
          write: ["angleSet"]
          read: ["angleAct"]
        command_topic: "/hand_left/angle_set"
        state_topic: "/hand_left/angle_actual"

    services:
      - register_name: "angleSet"
        set_service_name: "/hand_left/set_angle"
        is_write_register: true
```

## 자주 묻는 질문

### 1. 의존성 설치 문제

#### CMake 버전이 너무 낮음

```bash
cmake --version

# 3.10 미만이면 CMake 업그레이드
# Ubuntu 22.04 기본 CMake 는 일반적으로 요구사항 충족
# 업그레이드 필요 시 소스 빌드 또는 snap 사용
sudo snap install cmake --classic
```

#### Boost 라이브러리를 찾을 수 없음

```bash
pkg-config --modversion boost

# 찾을 수 없으면 재설치
sudo apt install --reinstall libboost-system-dev libboost-thread-dev libboost-dev

# 라이브러리 파일 위치 확인
dpkg -L libboost-system-dev | grep .so
```

#### yaml-cpp 라이브러리를 찾을 수 없음

```bash
pkg-config --modversion yaml-cpp

# 찾을 수 없으면 재설치
sudo apt install --reinstall libyaml-cpp-dev

# 라이브러리 파일 위치 확인
dpkg -L libyaml-cpp-dev | grep .so
```

#### spdlog 라이브러리를 찾을 수 없음

```bash
# 방법 1: apt 로 설치 (권장)
sudo apt install libspdlog-dev

# 방법 2: 소스에서 빌드
cd /tmp
git clone https://github.com/gabime/spdlog.git
cd spdlog
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)
sudo make install
sudo ldconfig
```

#### ROS2 미설치 또는 버전 불일치

```bash
echo $ROS_DISTRO

# 설정 없으면 ROS2 Humble 설치
# 위의 "ROS2 의존성 설치" 섹션 참조

# 버전 불일치 시 구 버전 제거 후 재설치
```

#### 빌드 시 헤더 파일을 찾을 수 없음

```bash
# 라이브러리 헤더 파일 위치 확인
dpkg -L libboost-dev | grep include
dpkg -L libyaml-cpp-dev | grep include

# 찾을 수 없으면 개발 패키지 재설치
sudo apt install --reinstall libboost-dev libyaml-cpp-dev
```

### 2. 시리얼 포트 권한 문제

```bash
# 사용자를 dialout 그룹에 추가
sudo usermod -a -G dialout $USER

# 재로그인 후 적용, 또는 즉시 적용
newgrp dialout

# 권한 확인
groups | grep dialout

# 또는 임시 권한 설정
sudo chmod 666 /dev/ttyUSB0
```

### 3. 디바이스를 찾을 수 없음

- 시리얼 디바이스 확인: `ls -l /dev/ttyUSB*`
- 설정 파일의 포트 경로 확인
- 디바이스 연결 확인
- USB-시리얼 드라이버 확인: `lsmod | grep usbserial`

### 4. 통신 타임아웃

- 보드레이트 설정 확인
- 디바이스 ID (Hand_ID) 설정 확인
- 시리얼 연결 확인
- 로그 파일에서 문제 점검
- 시리얼 포트가 다른 프로그램에 점유되지 않았는지 확인: `lsof /dev/ttyUSB0`

### 5. ROS2 노드가 시작되지 않음

- 설정 파일 경로 확인
- ROS2 환경 확인: `source install/setup.bash`
- ROS2 패키지 빌드 여부 확인: `colcon list`
- 로그 확인: `ros2 run inspire_control_ros2 inspire_control_node --ros-args --log-level debug`
- 노드 실행 여부 확인: `ros2 node list`

### 6. 빌드 오류

#### ROS2 패키지를 찾을 수 없음

```bash
# ROS2 환경 source 확인
source /opt/ros/humble/setup.bash

# ROS2 패키지 확인
ros2 pkg list | grep rclcpp
```

#### 링크 오류

```bash
# 라이브러리 파일 존재 확인
ldconfig -p | grep boost
ldconfig -p | grep yaml
ldconfig -p | grep spdlog

# 동적 라이브러리 캐시 업데이트
sudo ldconfig
```

#### CMake 가 패키지를 찾을 수 없음

```bash
# pkg-config 경로 확인
echo $PKG_CONFIG_PATH

# 비어 있으면 기본 경로 추가
export PKG_CONFIG_PATH=/usr/lib/pkgconfig:/usr/local/lib/pkgconfig
```

## 확장 개발

### 새 프로토콜 추가

1. 새 프로토콜 클래스 생성, `Protocol` 상속
2. 모든 순수 가상 함수 구현
3. `REGISTER_PROTOCOL` 매크로로 등록
4. 설정 파일에 프로토콜 타입 지정

### 새 레지스터 추가

1. 프로토콜 클래스의 `REGISTER_MAP` 에 레지스터 주소 (및 읽기 길이 등) 추가
2. 해당 모델의 interfaces 패키지에 전용 `srv`/`msg` 추가 (외부 노출 시)
3. **`(device)_interface_adapter.cpp`** 에서 해당 레지스터 연결
4. **`ros2_controller_config.yaml`** 에 `topics` 또는 `services` 항목 추가

### 새 디바이스 추가

1. `device_protocol_config.yaml` 에 디바이스 설정 추가
2. `ros2_controller_config.yaml` 에 디바이스 노드 설정 추가
3. 시스템이 자동 인식 및 시작

---

**문서 버전**: v1.0
**최종 업데이트**: 2026-05-12
