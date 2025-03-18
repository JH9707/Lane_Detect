# RC카 차선인식 및 차선이탈방지 프로젝트

## 프로젝트 개요
이 프로젝트는 Arduino와 Raspberry Pi를 이용하여 RC카를 제어하는 시스템을 구현합니다.  
- **수동 제어:** 시리얼 통신을 통해 키보드 입력으로 차량을 전진, 후진, 좌/우 회전 및 정지시킵니다.  
- **자율 주행:** 외부에서 전송한 각도값에 따라 차량의 조향을 자동으로 제어합니다.

## 프로젝트 기간
#### 24. 12. 26. ~ 25. 01. 07. (총 13일)



## 하드웨어 구성

<img src="/Images/RC_CAR.jpg" width="30%" height="25%">
<img src="/Images/Diagram.jpg" width="80%" height="80%">


- **Raspberry Pi:** 자율 주행 영상 처리 및 제어 명령 전송
- **Arduino Uno:** RC 차량 제어를 위한 마이크로컨트롤러
- **L298N:** 모터 제어를 위한 드라이버
- **카메라 (PiCamera 등):** 차량 전면에 부착되어 차선 검출
- **RC카 플랫폼:** 차량 프레임, DC모터, 바퀴 등

## 주요 기능
- **시리얼 통신:** Arduino가 외부에서 전달받은 명령을 처리
- **모터 제어:** 전진, 후진, 좌회전, 우회전, 정지 기능 구현
- **차선 검출 및 인식:** Raspberry Pi와 카메라를 이용해 실시간으로 차선을 검출 후, 결과를 바탕으로 자율 주행 제어
- **각도 기반 제어:** 'A' 명령으로 전송된 각도값을 바탕으로 조향 제어
- **일시정지/재개:** 'p' 명령을 통해 차량 동작을 일시정지 / 재개


## 파일 구조
#### ├── Images
#### ├── arduino.cpp // Arduino 코드 (RC 차량 제어)
#### ├── main.py // Raspberry Pi용 자율 주행 및 제어 코드 
#### └── README.md // 프로젝트 개요 및 사용법 문서

## 설치 및 실행 방법

### Arduino 코드 업로드
1. Arduino IDE를 실행하고 `arduino.cpp` 파일을 스케치에 넣습니다.
2. 보드를 Arduino Uno로 포트를 /dev/ttyACM0로 설정합니다.
3. 스케치를 Arduino 보드에 업로드합니다.

### main.py 실행
1. main.py 파일을 실행합니다.
2. 터미널에서 제어 명령(예: w, s, a, d, p, q 등)을 입력하여 차량을 제어합니다.

### 제어 명령 설명
- w: 전진
- s: 후진
- a: 좌회전
- d: 우회전
- x: 차량 정지
- p: 차량 일시정지/재개
- q: 프로그램 종료
