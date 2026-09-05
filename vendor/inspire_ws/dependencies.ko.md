# 의존성 목록

본 문서는 프로젝트에 필요한 의존성을 나열합니다: **시스템 / apt 로 설치하는 라이브러리** 와 **본 저장소와 함께 빌드되는 ROS2 인터페이스 패키지**.

## 시스템 요구 사항

- **OS**: Ubuntu 22.04+ (스크립트 `install_dependencies.sh` 는 Debian/Ubuntu 대상)
- **아키텍처**: x86_64 / amd64 (일반)

## 빌드 도구

| 의존성 | 버전 요구 | 설치 명령 |
|--------|----------|-----------|
| CMake | >= 3.10 | `sudo apt install cmake` |
| GCC/G++ | >= 9 | `sudo apt install gcc g++` |
| Make | 최신 | `sudo apt install make` |
| pkg-config | 최신 | `sudo apt install pkg-config` |

## ROS2 의존성 (apt / 공식 소스)

워크스페이스 빌드 및 실행에는 ROS2 기본 환경 필요 (**Humble** 예시, 실제 distro 에 따라 경로와 패키지 접두사 교체).

| 의존성 | 설명 | 일반 설치 명령 |
|--------|------|---------------|
| ROS2 Humble (또는 Jazzy 등) | 데스크톱 또는 간소화 버전 모두 가능 | `sudo apt install ros-humble-desktop` |
| rclcpp | C++ 클라이언트 라이브러리 | `sudo apt install ros-humble-rclcpp` |
| std_msgs | 표준 메시지 | `sudo apt install ros-humble-std-msgs` |
| rosidl_default_generators | 인터페이스 코드 생성 | `sudo apt install ros-humble-rosidl-default-generators` |
| rosidl_default_runtime | 인터페이스 런타임 | `sudo apt install ros-humble-rosidl-default-runtime` |
| colcon | 워크스페이스 빌드 | `sudo apt install python3-colcon-common-extensions` |
| rosdep | 의존성 해석 (선택) | `sudo apt install python3-rosdep` |

참고: **ament_index_cpp**, **builtin_interfaces** 등은 보통 `rclcpp` / 데스크톱 메타 패키지와 함께 설치되므로 별도 명시 불필요.

## 워크스페이스 내 ROS2 패키지 (소스 빌드, apt 아님)

본 저장소의 `src/ros2/src/` 하위에 다음 패키지가 있으며, **`inspire_control_ros2`** 와 같은 워크스페이스에서 **colcon** 으로 빌드해야 합니다:

| 패키지 이름 | 경로 (저장소 내) | 설명 |
|------------|-----------------|------|
| **rh5dg2_interfaces** | `interfaces/RH5DG2` | RH5DG2 (13 자유도) 전용 `.msg` / `.srv` |
| **rh56f1_interfaces** | `interfaces/RH56F1` | RH56 시리즈 (6 자유도) 전용 `.msg` / `.srv` |
| **inspire_control_ros2** | `driver` | 노드 실행 파일 `inspire_control_node`, 위의 두 인터페이스 패키지에 의존 |

**선언 관계** ( `src/ros2/src/driver/package.xml` 참조): `inspire_control_ros2` **depend** `rh5dg2_interfaces`, `rh56f1_interfaces`, `rclcpp`, `std_msgs`.

권장 최초 빌드 명령:

```bash
cd /path/to/serial_control/src/ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select rh5dg2_interfaces rh56f1_interfaces inspire_control_ros2
source install/setup.bash
```

## 서드파티 라이브러리 의존성 (apt)

| 의존성 | 버전 요구 | 설치 명령 | 용도 |
|--------|----------|-----------|------|
| Boost (system) | >= 1.65 | `sudo apt install libboost-system-dev` | 비동기 시리얼 (Asio) |
| Boost (thread) | >= 1.65 | `sudo apt install libboost-thread-dev` | 스레드 |
| Boost (dev 메타) | >= 1.65 | `sudo apt install libboost-dev` | 헤더 및 CMake |
| yaml-cpp | >= 0.6 | `sudo apt install libyaml-cpp-dev` | YAML 설정 파싱 |
| spdlog | >= 1.5 | `sudo apt install libspdlog-dev` | 로그 |

## 시스템 권한

| 권한 항목 | 설명 | 설정 명령 |
|----------|------|-----------|
| dialout 그룹 | `/dev/ttyUSB*` 등 시리얼 포트 접근 | `sudo usermod -a -G dialout $USER` |

## 빠른 설치

### 방법 1: 설치 스크립트 사용 (권장)

```bash
./install_dependencies.sh
```

스크립트는 **CMake/GCC/Boost/yaml-cpp/spdlog** 와 시리얼 포트 그룹을 설정합니다. **ROS2 미설치 시** 공식 설치 가이드를 출력합니다. 인터페이스 패키지와 노드 패키지는 ROS2 설치 후 워크스페이스에서 **colcon** 으로 빌드해야 합니다.

### 방법 2: 수동 설치

[README.ko.md](README.ko.md) 의 「의존성 설치」 섹션 참조. 서드파티 라이브러리 명령은 위 표와 동일합니다.

## 설치 검증

```bash
cmake --version
gcc --version
pkg-config --modversion boost
pkg-config --modversion yaml-cpp
dpkg -l | grep spdlog

echo $ROS_DISTRO
ros2 pkg prefix rclcpp

groups | grep dialout
```

빌드 검증:

```bash
cd src/ros2 && source /opt/ros/humble/setup.bash
colcon build --packages-select rh5dg2_interfaces rh56f1_interfaces inspire_control_ros2
```

## 의존성 관계도 (개념)

```
serial_control
├── 빌드 시스템: CMake / GCC / Make / pkg-config
├── ROS2 (시스템): rclcpp, std_msgs, rosidl 툴체인, colcon
├── 워크스페이스 ROS 패키지 (소스)
│   ├── rh5dg2_interfaces ──┐
│   ├── rh56f1_interfaces ──┼──► inspire_control_ros2 (노드)
│   └── (device 프로토콜 스택 .cpp 가 Boost/yaml-cpp/spdlog 링크)
├── 서드파티 라이브러리: Boost, yaml-cpp, spdlog
└── 권한: dialout (시리얼 포트)
```

## 자주 묻는 질문

### Q: `inspire_control_ros2` 만 빌드하면 `rh5dg2_interfaces` 를 찾을 수 없다는 오류가 발생?

먼저 같은 워크스페이스에서 두 인터페이스 패키지를 빌드하거나 다음을 사용:

`colcon build --packages-select rh5dg2_interfaces rh56f1_interfaces inspire_control_ros2`

### Q: 토픽 타입이 `RegisterData` 가 아닙니까?

리팩토링 이후 메시지 정의는 **`rh5dg2_interfaces` / `rh56f1_interfaces`** 에 있으며, **`interfaces_profile`** 로 선택됩니다. `ros2 topic info`, `ros2 interface list -p` 로 확인하세요.

---

**문서 버전**: v1.0  
**최종 업데이트**: 2026-05-12
