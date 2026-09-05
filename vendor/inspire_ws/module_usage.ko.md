# 정밀 핸드 제어 프로젝트 — 모듈 사용 설명서

본 문서는 프로젝트 각 모듈의 기능, 사용 방법, 설정 파라미터, 사용 예제를 상세히 설명합니다.

---

## 목차

1. [시리얼 통신 모듈 (SerialPortBase)](#1-시리얼-통신-모듈-serialportbase)
2. [프로토콜 추상 계층 (Protocol)](#2-프로토콜-추상-계층-protocol)
3. [디바이스 매니저 (DeviceManager)](#3-디바이스-매니저-devicemanager)
4. [설정 로더 (ConfigLoader)](#4-설정-로더-configloader)
5. [로그 매니저 (LoggerManager)](#5-로그-매니저-loggermanager)
6. [링 버퍼 (RingBuffer)](#6-링-버퍼-ringbuffer)
7. [프로토콜 팩토리 (ProtocolFactory)](#7-프로토콜-팩토리-protocolfactory)
8. [ROS2 컨트롤러 (RegisterController)](#8-ros2-컨트롤러-registercontroller)
9. [ROS2 메인 노드 (inspire_control_node)](#9-ros2-메인-노드-inspire_control_node)

---

## 1. 시리얼 통신 모듈 (SerialPortBase)

### 1.1 기능 소개

`SerialPortBase` 는 시리얼 통신의 추상 기본 클래스로, Boost.Asio 기반의 비동기 시리얼 통신을 구현합니다. 블로킹 읽기/쓰기 인터페이스를 제공하고 타임아웃 메커니즘을 지원하며, 내부적으로 비동기 수신 스레드로 데이터를 지속 수신합니다.

### 1.2 주요 클래스와 함수

#### 클래스 정의

```cpp
class SerialPortBase {
public:
    SerialPortBase(const std::string& port, unsigned int baudrate);
    ~SerialPortBase();
    
    // 블로킹 쓰기
    size_t write(const std::vector<uint8_t>& data);
    
    // 블로킹 읽기 (타임아웃 포함)
    std::vector<uint8_t> read(std::chrono::milliseconds timeout = std::chrono::milliseconds(25));
    
    // 비블로킹 읽기 (현재 버퍼 데이터 반환)
    std::vector<uint8_t> readAvailable();
    
    // 수신 버퍼 비우기
    void clearBuffer();
    
    // 시리얼 포트 상태 확인
    bool isOpen() const;
};
```

### 1.3 설정 파라미터

**생성자 파라미터**:
- `port`: 시리얼 디바이스 경로 (예: `/dev/ttyUSB0`)
- `baudrate`: 보드레이트 (예: 115200)

**내부 파라미터**:
- `BUFFER_SIZE = 4096`: 수신 버퍼 크기

### 1.4 사용 예제

```cpp
#include "serial_port.hpp"

// 시리얼 객체 생성
auto serial = std::make_shared<SerialPortBase>("/dev/ttyUSB0", 115200);

// 시리얼 포트 오픈 확인
if (!serial->isOpen()) {
    std::cerr << "시리얼 포트 오픈 실패" << std::endl;
    return;
}

// 데이터 쓰기
std::vector<uint8_t> cmd = {0xEB, 0x90, 0x01, 0x04, 0x11, 0x28, 0x04, 0x0C, 0xXX};
size_t written = serial->write(cmd);

// 데이터 읽기 (블로킹, 타임아웃 25ms)
auto data = serial->read(std::chrono::milliseconds(25));

// 비블로킹 읽기 (현재 버퍼 데이터 즉시 반환)
auto available_data = serial->readAvailable();

// 수신 버퍼 비우기
serial->clearBuffer();
```

### 1.5 데이터 흐름과 통신 방식

**쓰기 흐름**:
```
write() 호출 → Boost.Asio 동기 쓰기 → 시리얼 하드웨어
```

**읽기 흐름**:
```
비동기 수신 스레드 → 데이터 지속 수신 → receive_buffer_ 에 저장
read() 호출 → receive_buffer_ 에서 읽기 → 데이터 반환
```

**스레드 모델**:
- 메인 스레드: `write()` 및 `read()` 호출
- 비동기 스레드: 시리얼 데이터 지속 수신, 버퍼에 저장

---

## 2. 프로토콜 추상 계층 (Protocol)

### 2.1 기능 소개

`Protocol` 은 프로토콜 추상 기본 클래스로, 모든 프로토콜 구현이 따라야 하는 인터페이스 사양을 정의합니다. 구체 프로토콜 (예: `RH56F1_485_Protocol`) 은 이 클래스를 상속하고 프로토콜별 명령 빌드와 응답 파싱 로직을 구현합니다.

### 2.2 주요 클래스와 함수

#### 기본 클래스 인터페이스

```cpp
class Protocol {
public:
    // 디바이스 ID 설정/조회
    void setDeviceId(uint8_t id);
    uint8_t getDeviceId() const;
    
    // 레지스터 주소 조회
    virtual int getRegisterAddress(const std::string& register_name) const = 0;
    
    // 명령 빌드
    virtual std::vector<uint8_t> buildReadCommand(int address, size_t length) = 0;
    virtual std::vector<uint8_t> buildWriteCommand(int address, const std::vector<int>& values) = 0;
    
    // 응답 파싱
    virtual std::pair<bool, std::vector<int>> parseResponse(RingBuffer& ringBuffer) = 0;
    virtual bool validateChecksum(const std::vector<uint8_t>& response) const = 0;
    
    // 촉각 데이터
    virtual std::pair<bool, TouchDataResult> parseTouchData(RingBuffer& ringBuffer, int version) = 0;
    
    // 고급 인터페이스
    virtual bool writeRegister(Device device, const std::string& reg_name, const std::vector<int>& values) = 0;
    virtual std::pair<bool, std::vector<int>> readRegister(Device device, RingBuffer& ringBuffer, const std::string& reg_name, size_t length) = 0;
    virtual std::pair<bool, TouchDataResult> readTouchData(Device device, RingBuffer& ringBuffer, int version) = 0;
};
```

#### 구체 프로토콜 구현

**RH56F1_485_Protocol**:
- RH56F1 시리즈 디바이스의 485 통신 프로토콜 구현
- 동적 읽기 길이 지원
- 촉각 데이터 파싱 지원 (버전 1, 2)

**RH5DG2_485_Protocol**:
- RH5DG2 시리즈 디바이스의 485 통신 프로토콜 구현
- RH56F1 프로토콜과 유사하나 레지스터 주소가 다름
- `actionLibraryIndex` 특수 처리 지원

### 2.3 설정 파라미터

**디바이스 ID (Hand_ID)**:
- 범위: 1–254
- 기본값: 1
- 용도: 동일 시리얼 버스의 여러 디바이스 구분

**레지스터 매핑**:
- 각 프로토콜 클래스는 `REGISTER_MAP` 을 유지하며 레지스터 이름을 주소에 매핑
- 예: `{"angleSet": 1040, "angleAct": 1064}`

**읽기 길이 매핑**:
- `REGISTER_READ_LENGTH_MAP`: 레지스터별 기본 읽기 길이 정의
- 예: `{"angleAct": 12, "errorCode": 2}`

### 2.4 사용 예제

```cpp
#include "RH56F1_485_protocol.hpp"
#include "serial_port.hpp"
#include "ring_buffer.hpp"

// 프로토콜 객체 생성
auto protocol = std::make_shared<RH56F1_485_Protocol>();

// 디바이스 ID 설정
protocol->setDeviceId(1);

// 시리얼 포트와 버퍼 생성
auto device = std::make_shared<SerialPortBase>("/dev/ttyUSB0", 115200);
RingBuffer ringBuffer(1024);

// 레지스터 쓰기
std::vector<int> values = {100, 200, 300, 400, 500, 600};
bool success = protocol->writeRegister(device, "angleSet", values);

// 레지스터 읽기 (length=0 은 자동 길이 결정)
auto [success, values] = protocol->readRegister(device, ringBuffer, "angleAct", 0);

// 촉각 데이터 읽기
auto [success, touchData] = protocol->readTouchData(device, ringBuffer, 1);
```

### 2.5 데이터 흐름과 통신 방식

**쓰기 흐름**:
```
writeRegister() 
  → buildWriteCommand() (명령 프레임 빌드)
  → device->write() (시리얼로 송신)
  → readResponseWithLoop() (응답 읽기)
  → parseResponse() (응답 파싱)
  → 성공/실패 반환
```

**읽기 흐름**:
```
readRegister() 
  → buildReadCommand() (명령 프레임 빌드)
  → device->write() (시리얼로 송신)
  → readResponseWithLoop() (응답 읽기)
  → parseResponse() (응답 파싱)
  → 값과 성공 플래그 반환
```

**프레임 형식**:
- 요청 프레임: `0xEB 0x90` + 디바이스 ID + 데이터 길이 + 명령 + 주소 + 데이터 + 체크섬
- 응답 프레임: `0x90 0xEB` + 디바이스 ID + 데이터 길이 + 명령 + 주소 + 데이터 + 체크섬

---

## 3. 디바이스 매니저 (DeviceManager)

### 3.1 기능 소개

`DeviceManager` 는 여러 시리얼 디바이스를 관리하며 포트와 디바이스 객체 매핑 관계를 유지합니다. 각 디바이스는 하나의 `SerialPortBase` 객체에 대응됩니다.

### 3.2 주요 클래스와 함수

```cpp
class DeviceManager {
public:
    // 디바이스 추가
    void addDevice(const std::string& port, std::shared_ptr<Protocol> protocol, int baudRate);
    
    // 디바이스 조회
    std::shared_ptr<SerialPortBase> getDevice(const std::string& port);
    
    // 디바이스 제거
    void removeDevice(const std::string& port);
};
```

### 3.3 설정 파라미터

**addDevice 파라미터**:
- `port`: 시리얼 디바이스 경로 (예: `/dev/ttyUSB0`)
- `protocol`: 프로토콜 객체 포인터 (디바이스 프로토콜 설정용)
- `baudRate`: 보드레이트 (예: 115200)

### 3.4 사용 예제

```cpp
#include "device_manager.hpp"
#include "RH56F1_485_protocol.hpp"

// 디바이스 매니저 생성
DeviceManager device_manager;

// 프로토콜 객체 생성
auto protocol = std::make_shared<RH56F1_485_Protocol>();
protocol->setDeviceId(1);

// 디바이스 추가
device_manager.addDevice("/dev/ttyUSB0", protocol, 115200);

// 디바이스 조회
auto device = device_manager.getDevice("/dev/ttyUSB0");
if (device) {
    // 디바이스 사용
    std::vector<uint8_t> cmd = {0xEB, 0x90, 0x01, 0x04, 0x11, 0x28, 0x04, 0x0C, 0xXX};
    device->write(cmd);
}

// 디바이스 제거
device_manager.removeDevice("/dev/ttyUSB0");
```

### 3.5 데이터 흐름과 통신 방식

**디바이스 관리 흐름**:
```
addDevice() 
  → SerialPortBase 객체 생성
  → 프로토콜 설정
  → devices 매핑 테이블에 저장
  → 비동기 수신 스레드 시작
```

**디바이스 조회 흐름**:
```
getDevice(port) 
  → devices 매핑 테이블에서 검색
  → SerialPortBase 객체 반환
```

---

## 4. 설정 로더 (ConfigLoader)

### 4.1 기능 소개

`ConfigLoader` 는 YAML 설정 파일에서 디바이스 설정을 로드하고, 프로토콜 객체를 생성하며, 로그 시스템을 설정하는 역할을 합니다.

### 4.2 주요 클래스와 함수

```cpp
class ConfigLoader {
public:
    // 디바이스 설정 로드
    static std::unordered_map<std::string, DeviceInfo> loadDeviceConfig(const std::string& config_path);
    
    // 프로토콜 객체 생성
    static std::shared_ptr<Protocol> createProtocolFromConfig(const std::string& config_path);
    
    // 로그 시스템 설정
    static void configureLogging(const std::string& config_path);
};
```

### 4.3 설정 파라미터

**설정 파일 형식 (device_protocol_config.yaml)**:

```yaml
protocol:
  type: RH56F1_485  # 프로토콜 타입

devices:
  - name: hand_left      # 디바이스 이름
    port: /dev/ttyUSB0   # 시리얼 포트
    baudrate: 115200     # 보드레이트
    Hand_ID: 1           # 디바이스 ID

logging:
  level: DEBUG           # 로그 레벨
  file: logs/hand_control.log  # 로그 파일 경로
  console: true          # 콘솔 출력 여부
  file_enable: true      # 파일 출력 여부
  max_file_size_mb: 10   # 단일 로그 파일 최대 크기 (MB)
  max_files: 5           # 보존할 로그 파일 개수
```

### 4.4 사용 예제

```cpp
#include "config_loader.hpp"
#include "device_manager.hpp"

// 로그 시스템 설정
ConfigLoader::configureLogging("config/device_protocol_config.yaml");

// 디바이스 설정 로드
auto deviceConfig = ConfigLoader::loadDeviceConfig("config/device_protocol_config.yaml");

// 디바이스 설정 순회
for (const auto& [port, deviceInfo] : deviceConfig) {
    std::cout << "디바이스: " << deviceInfo.name 
              << ", 포트: " << port 
              << ", 보드레이트: " << deviceInfo.baudrate 
              << ", Hand_ID: " << deviceInfo.hand_id << std::endl;
}

// 프로토콜 객체 생성
auto protocol = ConfigLoader::createProtocolFromConfig("config/device_protocol_config.yaml");
```

### 4.5 데이터 흐름과 통신 방식

**설정 로드 흐름**:
```
loadDeviceConfig() 
  → YAML 파일 읽기
  → 디바이스 리스트 파싱
  → DeviceInfo 매핑 테이블 반환
```

**프로토콜 생성 흐름**:
```
createProtocolFromConfig() 
  → YAML 파일 읽기
  → 프로토콜 타입 조회
  → ProtocolFactory::create()
  → 프로토콜 객체 반환
```

---

## 5. 로그 매니저 (LoggerManager)

### 5.1 기능 소개

`LoggerManager` 는 spdlog 기반의 통합된 로그 관리 기능을 제공합니다. 콘솔 및 파일 출력을 지원하며, 로그 회전과 레벨 제어를 지원합니다.

### 5.2 주요 클래스와 함수

```cpp
class LoggerManager {
public:
    // 로그 시스템 초기화 (설정 구조체로부터)
    static void initialize(const LogConfig& config);
    
    // 로그 시스템 초기화 (YAML 노드로부터)
    static void initialize(const YAML::Node& logging_node);
    
    // 동적으로 로그 레벨 설정
    static void setLogLevel(const std::string& level);
    static void setLogLevel(spdlog::level::level_enum level);
    
    // 로그 레벨 조회
    static std::string getLogLevel();
    
    // 로그 객체 조회
    static std::shared_ptr<spdlog::logger> getLogger();
    
    // 로그 시스템 재설정
    static void reconfigure(const LogConfig& config);
    
    // 로그 버퍼 플러시
    static void flush();
    
    // 로그 시스템 종료
    static void shutdown();
};
```

### 5.3 설정 파라미터

**LogConfig 구조체**:
```cpp
struct LogConfig {
    std::string level = "INFO";              // 로그 레벨
    std::string file_path = "logs/app.log";  // 로그 파일 경로
    bool console_enable = true;              // 콘솔 출력 여부
    bool file_enable = true;                 // 파일 출력 여부
    size_t max_file_size = 10 * 1024 * 1024; // 단일 로그 파일 최대 크기 (바이트)
    size_t max_files = 5;                    // 보존할 로그 파일 개수
};
```

**로그 레벨**:
- `TRACE`: 가장 상세한 로그
- `DEBUG`: 디버그 정보
- `INFO`: 일반 정보
- `WARN`: 경고
- `ERROR`: 오류
- `CRITICAL`: 치명적 오류
- `OFF`: 로그 비활성화

### 5.4 사용 예제

```cpp
#include "logger_manager.hpp"

// 방법 1: YAML 설정으로 초기화
YAML::Node config = YAML::LoadFile("config/device_protocol_config.yaml");
LoggerManager::initialize(config["logging"]);

// 방법 2: 설정 구조체로 초기화
LoggerManager::LogConfig log_config;
log_config.level = "DEBUG";
log_config.file_path = "logs/app.log";
log_config.console_enable = true;
log_config.file_enable = true;
log_config.max_file_size = 10 * 1024 * 1024;
log_config.max_files = 5;
LoggerManager::initialize(log_config);

// 로그 객체 조회 및 사용
auto logger = getLogger();
logger->info("정보 로그입니다");
logger->debug("디버그 로그입니다");
logger->warn("경고 로그입니다");
logger->error("오류 로그입니다");

// 동적으로 로그 레벨 조정
LoggerManager::setLogLevel("WARN");

// 로그 버퍼 플러시
LoggerManager::flush();

// 로그 시스템 종료 (프로그램 종료 시)
LoggerManager::shutdown();
```

### 5.5 데이터 흐름과 통신 방식

**로그 출력 흐름**:
```
logger->info() 
  → spdlog 로그 시스템
  → 콘솔 sink (활성화 시)
  → 파일 sink (활성화 시)
  → 로그 파일 (자동 회전)
```

**로그 회전**:
- 로그 파일이 `max_file_size` 에 도달하면 자동으로 새 파일 생성
- `max_files` 개의 로그 파일 보존, 초과 시 가장 오래된 것 삭제

---

## 6. 링 버퍼 (RingBuffer)

### 6.1 기능 소개

`RingBuffer` 는 시리얼 수신 데이터를 캐싱하는 링 버퍼입니다. 효율적인 데이터 enqueue/dequeue 연산을 지원하며, 링 구조를 파괴하지 않고 지정된 위치에서 데이터를 읽을 수 있습니다.

### 6.2 주요 클래스와 함수

```cpp
class RingBuffer {
public:
    explicit RingBuffer(size_t size);
    
    // 버퍼 비우기
    void clear();
    
    // enqueue: 링 버퍼에 데이터 추가
    void push(const uint8_t* data, size_t len);
    
    // dequeue: 버퍼에서 데이터 읽기
    size_t pop(uint8_t* data, size_t maxlen);
    
    // 현재 버퍼의 유효 데이터 길이 조회
    size_t size() const;
    
    // tail 부터 연속된 데이터 길이 조회
    size_t contiguousDataSize() const;
    
    // 버퍼의 하위 데이터 포인터 조회
    const uint8_t* data() const;
    
    // tail 부터 연속된 영역의 포인터 조회
    const uint8_t* dataPtr() const;
    
    // 파싱된 데이터 트림, tail 포인터 이동
    void advance(size_t count);
    
    // buffer 하위 데이터 조회 (상수 참조)
    const std::vector<uint8_t>& getBuffer() const;
    
    // tail 인덱스 조회
    size_t getTail() const;
};
```

### 6.3 설정 파라미터

**생성자 파라미터**:
- `size`: 버퍼 크기 (예: 1024)

### 6.4 사용 예제

```cpp
#include "ring_buffer.hpp"

// 링 버퍼 생성
RingBuffer ringBuffer(1024);

// 데이터 추가
uint8_t data[] = {0x90, 0xEB, 0x01, 0x0F, 0x11, 0x28, 0x04, 0x64, 0x00};
ringBuffer.push(data, sizeof(data));

// 데이터 길이 조회
size_t data_size = ringBuffer.size();

// 데이터 읽기 (링 구조 파괴 없이)
uint8_t read_buffer[1024];
size_t read_len = ringBuffer.pop(read_buffer, sizeof(read_buffer));

// 지정 위치에서 읽기 (프로토콜 파싱용)
size_t offset = 0;
uint8_t byte = readByteAtOffset(ringBuffer, offset);

// 파싱된 데이터 트림
ringBuffer.advance(10);  // tail 포인터 이동, 앞쪽 10 바이트 폐기

// 버퍼 비우기
ringBuffer.clear();
```

### 6.5 데이터 흐름과 통신 방식

**enqueue 흐름**:
```
push(data, len) 
  → 버퍼 공간 확인
  → head 위치에 데이터 쓰기
  → head 포인터 업데이트
```

**dequeue 흐름**:
```
pop(data, maxlen) 
  → tail 위치에서 데이터 읽기
  → tail 포인터 업데이트
  → 읽은 길이 반환
```

**트림 흐름**:
```
advance(count) 
  → tail 포인터 이동
  → 파싱된 데이터 공간 해제
```

---

## 7. 프로토콜 팩토리 (ProtocolFactory)

### 7.1 기능 소개

`ProtocolFactory` 는 팩토리 패턴과 등록 메커니즘을 사용하여 프로토콜 객체를 동적으로 등록하고 생성합니다. 새 프로토콜은 등록만 하면 되고, 코어 코드를 수정할 필요가 없습니다.

### 7.2 주요 클래스와 함수

```cpp
class ProtocolFactory {
public:
    // 프로토콜 타입 등록
    static void registerProtocol(const std::string& type, ProtocolCreator creator);
    
    // 프로토콜 객체 생성
    static std::shared_ptr<Protocol> create(const std::string& type);
    
    // 등록된 프로토콜 타입 리스트 조회
    static std::vector<std::string> getRegisteredTypes();
    
    // 프로토콜 타입 등록 여부 확인
    static bool isRegistered(const std::string& type);
};
```

### 7.3 설정 파라미터

**프로토콜 등록 매크로**:
```cpp
REGISTER_PROTOCOL("RH56F1_485", RH56F1_485_Protocol);
```

### 7.4 사용 예제

```cpp
#include "protocol_factory.hpp"

// 프로토콜 등록 여부 확인
if (ProtocolFactory::isRegistered("RH56F1_485")) {
    // 프로토콜 객체 생성
    auto protocol = ProtocolFactory::create("RH56F1_485");
    
    // 프로토콜 사용
    protocol->setDeviceId(1);
}

// 등록된 모든 프로토콜 타입 조회
auto types = ProtocolFactory::getRegisteredTypes();
for (const auto& type : types) {
    std::cout << "등록된 프로토콜: " << type << std::endl;
}
```

### 7.5 데이터 흐름과 통신 방식

**프로토콜 등록 흐름** (프로그램 시작 시 자동 실행):
```
REGISTER_PROTOCOL 매크로
  → 정적 전역 객체 생성
  → registerProtocol() 호출
  → 등록 테이블에 저장
```

**프로토콜 생성 흐름**:
```
create(type) 
  → 등록 테이블에서 생성 함수 검색
  → 생성 함수 호출
  → 프로토콜 객체 반환
```

---

## 8. ROS2 컨트롤러 (RegisterController)

### 8.1 기능 소개

`RegisterController` 는 ROS2 디바이스 제어 노드로, 디바이스 하나당 노드 하나가 대응됩니다. 외부 ROS 타입은 **`device_protocol_config.yaml`** 의 **`protocol.type`** 에서 자동 추론되어 **`RH5DG2` / `RH56F1`** 로 결정되고, **`InterfaceAdapter`** 를 통해 **`rh5dg2_interfaces` 또는 `rh56f1_interfaces`** 에 매핑됩니다 (통합된 `RegisterData` / `SetRegister` 는 더 이상 존재하지 않음). 토픽과 서비스를 지원하며, 타이머 루프로 레지스터를 읽고 상태를 발행합니다.

### 8.2 주요 클래스와 함수

```cpp
class RegisterController : public rclcpp::Node {
public:
    RegisterController(
        const std::string& node_name,
        const DeviceNodeConfig& config,
        std::shared_ptr<SerialPortBase> device,
        std::shared_ptr<Protocol> protocol
    );
    
    // 컨트롤러 초기화
    void initialize();
    
    // 제어 루프 시작
    void start();
    
    // 제어 루프 중지
    void stop();
    
protected:
    // 레지스터 값 읽기
    std::pair<bool, std::vector<int>> readRegister(const std::string& reg_name, size_t length = 0);
    
    // 레지스터 값 쓰기
    bool writeRegister(const std::string& reg_name, const std::vector<int>& values);
    
    // 촉각 데이터 읽기
    std::pair<bool, TouchDataResult> readTouchData(int version = 0);
    
    // 제어 루프 (타이머 호출)
    void controlLoop();
};
```

### 8.3 설정 파라미터

**DeviceNodeConfig 구조체**:
```cpp
struct DeviceNodeConfig {
    std::string device_name;              // 디바이스 이름
    std::string interfaces_profile;       // protocol.type 으로부터 추론 (RH5DG2 / RH56F1), ros2_controller_config.yaml 에서 설정 금지
    std::string publish_frame_id;         // publish_header.frame_id 또는 frame_id 로부터, 발행 메시지의 Header 에 기록
    std::vector<std::string> joint_names; // 모델의 자유도와 일치 (RH5DG2:13 / RH56F1:6), joint_names 배열에 기록
    std::vector<TopicConfig> topics;      // 토픽 설정 리스트
    std::vector<ServiceConfig> services;  // 서비스 설정 리스트
    double update_rate = 50.0;            // 타이머 업데이트 주파수 (Hz)
};
```

**TopicConfig 구조체**:
```cpp
struct TopicConfig {
    std::string name;                          // 토픽 이름 식별자
    std::vector<std::string> write_registers;  // 쓰기 레지스터 리스트
    std::vector<std::string> read_registers;   // 읽기 레지스터 리스트
    std::string command_topic;                 // 명령 토픽 이름
    std::string state_topic;                   // 상태 토픽 이름
    int touch_version = 1;                     // 촉각 데이터 버전 번호
};
```

**ServiceConfig 구조체**:
```cpp
struct ServiceConfig {
    std::string register_name;           // 레지스터 이름
    std::string set_service_name;        // Set 서비스 이름
    std::string get_service_name;        // Get 서비스 이름
    bool is_write_register;              // 쓰기 레지스터 여부
};
```

### 8.4 사용 예제

**설정 파일 (ros2_controller_config.yaml)**:

```yaml
device_nodes:
  - device: hand_left
    # interfaces_profile 은 device_protocol_config.yaml 의 protocol.type 으로부터 자동 추론, 여기 입력 금지
    update_rate: 50
    publish_header:
      frame_id: "hand_left"
    joint_names:
      - "hand_left/joint_0"
      # ... RH5DG2 총 13 개; RH56F1 총 6 개

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

아래는 **`protocol.type`** 이 RH5DG2 시리즈일 때의 명령 예제 (**RH56F1** 시리즈는 **`rh56f1_interfaces`** 사용, 관절 수는 **6**).

**ROS2 토픽 사용**:

```bash
# 각도 명령 발행 (13 차원 joint_values)
ros2 topic pub --once /hand_left/angle_set rh5dg2_interfaces/msg/SetAngle1 \
  "{hand_id: 1, joint_values: [100,100,100,100,100,100,100,100,100,100,100,100,100]}"

ros2 topic echo /hand_left/angle_actual
```

**ROS2 서비스 사용**:

```bash
ros2 service call /hand_left/set_angle rh5dg2_interfaces/srv/Setangle \
  "{command: '', hand_id: 1, joint_values: [100,100,100,100,100,100,100,100,100,100,100,100,100]}"

ros2 service call /hand_left/get_errorCode rh5dg2_interfaces/srv/Geterror \
  "{query: '', hand_id: 1}"
```

### 8.5 데이터 흐름과 통신 방식

**토픽 모드 데이터 흐름**:
```
타이머 루프 (50Hz)
  → controlLoop()
  → readRegister() (레지스터 읽기)
  → publishRegisterData() (상태 토픽 발행)
  
구독자가 명령 발행
  → Subscriber 콜백
  → handleCommand()
  → writeRegister() (레지스터 쓰기)
```

**서비스 모드 데이터 흐름**:
```
서비스 클라이언트 호출
  → Service 콜백
  → pauseTimer(3ms) (타이머 일시 정지)
  → readRegister() / writeRegister()
  → 서비스 응답 반환
  → 타이머 재개
```

---

## 9. ROS2 메인 노드 (inspire_control_node)

### 9.1 기능 소개

`inspire_control_node` 는 ROS2 메인 프로그램 진입점으로, ROS2 초기화, 설정 로드, 디바이스 및 컨트롤러 생성, ROS2 이벤트 루프 실행을 담당합니다.

### 9.2 주요 기능

1. **ROS2 초기화**: `rclcpp::init()`
2. **설정 파일 로드**: 디바이스 설정과 ROS2 컨트롤러 설정
3. **로그 시스템 설정**: 설정 파일에서 로그 설정 로드
4. **디바이스 매니저 생성**: 모든 시리얼 디바이스 관리
5. **프로토콜 객체 생성**: 디바이스마다 독립된 프로토콜 인스턴스
6. **컨트롤러 노드 생성**: 디바이스마다 RegisterController 노드 생성
7. **이벤트 루프 실행**: `MultiThreadedExecutor::spin_some()`

### 9.3 설정 파라미터

**명령줄 인자**:
- `--device-config`: 디바이스 프로토콜 설정 파일 경로
- `--controller-config`: ROS2 컨트롤러 설정 파일 경로
- `--device`: 단일 디바이스 모드, 디바이스 이름 지정

### 9.4 사용 예제

**ROS2 워크스페이스 빌드**:

```bash
cd /home/ubuntu/serial_control/src/ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select rh5dg2_interfaces rh56f1_interfaces inspire_control_ros2
source install/setup.bash
```

**단일 디바이스 노드 실행**:

```bash
# launch 파일 사용
ros2 launch inspire_control_ros2 inspire_control_single_device.launch.py \
  device_name:=hand_left

# 또는 노드 직접 실행
ros2 run inspire_control_ros2 inspire_control_node \
  --device-config /path/to/device_protocol_config.yaml \
  --controller-config /path/to/ros2_controller_config.yaml \
  --device hand_left
```

**다중 디바이스 노드 실행**:

```bash
# launch 파일 사용
ros2 launch inspire_control_ros2 inspire_control_multi_device.launch.py

# 또는 노드 직접 실행 (--device 인자 미지정)
ros2 run inspire_control_ros2 inspire_control_node \
  --device-config /path/to/device_protocol_config.yaml \
  --controller-config /path/to/ros2_controller_config.yaml
```

**노드 정보 조회**:

```bash
# 노드 리스트
ros2 node list

# 토픽 리스트
ros2 topic list

# 서비스 리스트
ros2 service list

# 노드 정보
ros2 node info /hand_left_node
```

### 9.5 데이터 흐름과 통신 방식

**시작 흐름**:
```
프로그램 시작
  → ROS2 초기화
  → 설정 파일 로드
  → 로그 시스템 설정
  → 디바이스 매니저 생성
  → 프로토콜 객체 생성
  → 컨트롤러 노드 생성
  → 타이머 루프 시작
  → ROS2 이벤트 루프 실행
```

**실행 흐름**:
```
ROS2 이벤트 루프
  ├─ 타이머 콜백 (50Hz)
  │  └─ controlLoop() (레지스터 읽기 및 발행)
  ├─ 토픽 구독 콜백
  │  └─ handleCommand() (레지스터 쓰기)
  └─ 서비스 콜백
     └─ 각 레지스터에 대응하는 전용 .srv (어댑터가 생성, 타입은 rh*_interfaces 참조)
```

---

## 10. 일반적인 사용 시나리오

### 10.1 단일 디바이스 제어

```bash
# 왼손 디바이스 노드 시작
ros2 launch inspire_control_ros2 inspire_control_single_device.launch.py \
  device_name:=hand_left

# 각도 제어 명령 발행 (RH5DG2: 13 차원)
ros2 topic pub --once /hand_left/angle_set rh5dg2_interfaces/msg/SetAngle1 \
  "{hand_id: 1, joint_values: [100,100,100,100,100,100,100,100,100,100,100,100,100]}"

# 각도 상태 구독
ros2 topic echo /hand_left/angle_actual
```

### 10.2 다중 디바이스 병렬 제어

```bash
# 모든 디바이스 노드 시작
ros2 launch inspire_control_ros2 inspire_control_multi_device.launch.py

# 왼손/오른손 동시 제어
ros2 topic pub --once /hand_left/angle_set rh5dg2_interfaces/msg/SetAngle1 \
  "{hand_id: 1, joint_values: [100,100,100,100,100,100,100,100,100,100,100,100,100]}"

ros2 topic pub --once /hand_right/angle_set rh5dg2_interfaces/msg/SetAngle1 \
  "{hand_id: 1, joint_values: [100,100,100,100,100,100,100,100,100,100,100,100,100]}"
```

### 10.3 서비스 호출

```bash
# 디바이스 통신 ID 설정
ros2 service call /hand_left/set_id rh5dg2_interfaces/srv/Setid \
  "{hand_id: 1, device_id: 2}"

# 오류 코드 조회
ros2 service call /hand_left/get_errorCode rh5dg2_interfaces/srv/Geterror \
  "{query: '', hand_id: 1}"

# 오류 클리어 (필드는 Setclearerror.srv 에 따름)
ros2 service call /hand_left/set_clearError rh5dg2_interfaces/srv/Setclearerror \
  "{hand_id: 1, clear_code: 1}"
```

### 10.4 촉각 데이터 읽기

```bash
# 촉각 데이터 토픽 구독
ros2 topic echo /hand_left/touch_data
```

---

## 11. 디버그와 장애 점검

### 11.1 로그 확인

```bash
# 로그 파일 확인
tail -f logs/hand_control.log

# ROS2 로그 확인
ros2 run inspire_control_ros2 inspire_control_node --ros-args --log-level debug
```

### 11.2 시리얼 포트 확인

```bash
# 시리얼 디바이스 확인
ls -l /dev/ttyUSB*

# 시리얼 권한 확인
sudo chmod 666 /dev/ttyUSB0

# 시리얼 통신 테스트
sudo minicom -D /dev/ttyUSB0 -b 115200
```

### 11.3 ROS2 노드 확인

```bash
# 노드 리스트
ros2 node list

# 토픽 리스트
ros2 topic list

# 서비스 리스트
ros2 service list

# 토픽 데이터 확인
ros2 topic echo /hand_left/angle_actual

# 서비스 타입 확인
ros2 service type /hand_left/set_angle
ros2 service type /hand_left/get_errorCode
```

---

**문서 버전**: v1.1 (강타입 인터페이스 패키지 및 inspire_control_ros2 정렬)  
**최종 업데이트**: 2026-05-12
