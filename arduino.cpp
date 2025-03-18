/*
  ---------------------------
  이 파일은 RC 차량 제어를 위한 Arduino 코드입니다.
  확장자가 .ino 가 아니기 때문에 직접 스케치에 붙여 넣고 컴파일 해야만 합니다.
  시리얼 통신을 통해 라즈베리파이로부터 명령을 수신하여, 차량의 모터를 제어합니다.
  
  주요 기능:
    - 시리얼로부터 문자 기반의 제어 명령 수신
    - 'x': 차량 정지
    - 'q': 프로그램 종료 (모든 동작 정지)
    - 'w', 's', 'a', 'd': 전진, 후진, 좌회전, 우회전 수동 제어 (일시정지 상태가 아닐 경우)
    - 'p': 차량 일시정지/재개 토글
    - 'A': 수신된 각도값에 따라 좌회전/우회전/전진 제어
*/

#define ENA 11   // 좌측 모터 속도 제어 (PWM)
#define IN1 10   // 좌측 모터 방향 제어 핀 1
#define IN2 9    // 좌측 모터 방향 제어 핀 2
#define IN3 8    // 우측 모터 방향 제어 핀 1
#define IN4 7    // 우측 모터 방향 제어 핀 2
#define ENB 6    // 우측모터B 속도 제어 (PWM)

int speed = 155;      // 모터 속도 (PWM 값)
char k = 0;           // 시리얼로부터 수신한 명령 저장 변수
int angle = 0;        // 수신한 각도 값 저장 변수 (A 명령용)
bool isPaused = false;  // 차량 일시정지 상태 플래그 (false: 동작 중, true: 정지 상태)

void setup() {
  // 모터 제어를 위한 핀을 출력 모드로 설정
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // 시리얼 통신 초기화 (115200 baud)
  Serial.begin(115200);
}

void loop() {
  // 시리얼 버퍼에 데이터가 있으면 읽어서 처리
  if (Serial.available() > 0) {
    k = Serial.read();

    // 개행 문자('\n', '\r')는 무시하여 처리하지 않음
    if (k == '\n' || k == '\r') {
      return;
    }

    // ------------------------------
    // 수신된 명령에 따른 제어 처리
    // ------------------------------

    // 'x': 차량 정지
    if (k == 'x') {
      stopCar();
      Serial.println("Car stopped.");
    }
    // 'q': 프로그램 종료 (무한 대기 상태)
    else if (k == 'q') {
      stopCar();
      Serial.println("Program exiting...");
      while (true) {
        delay(100);
      }
    }
    // 'w': 전진 (일시정지 상태가 아닐 때)
    else if (k == 'w' && !isPaused) {
      forward();
    }
    // 's': 후진 (일시정지 상태가 아닐 때)
    else if (k == 's' && !isPaused) {
      backward();
    }
    // 'a': 좌회전 (일시정지 상태가 아닐 때)
    else if (k == 'a' && !isPaused) {
      left();
    }
    // 'd': 우회전 (일시정지 상태가 아닐 때)
    else if (k == 'd' && !isPaused) {
      right();
    }
    // 'p': 차량 일시정지/재개 토글
    else if (k == 'p') {
      // isPaused 상태를 토글하고, 정지 상태일 경우 차량을 정지
      if (isPaused) {
        Serial.println("Resuming car movement...");
        isPaused = false;
      } else {
        stopCar();
        Serial.println("Car paused.");
        isPaused = true;
      }
    }
    // 'A': 수신된 각도에 따라 좌회전/우회전/전진 제어
    else if (k == 'A') {
      // 시리얼 버퍼에서 정수 형태의 각도값을 읽음
      angle = Serial.parseInt();
      Serial.print("Received angle: ");
      Serial.println(angle);

      // 각도 값이 임계값(10도)을 넘으면 좌/우 회전, 그 외에는 전진
      if (angle > 10 && !isPaused) {
        left();
      } else if (angle < -10 && !isPaused) {
        right();
      } else if (!isPaused) {
        forward();
      }
    }
    else {
      // 정의되지 않은 명령은 무시 (필요 시 디버깅용 출력 추가 가능)
      // Serial.println("Unknown command");
    }
  }
}

// ------------------------------
// 모터 제어 함수들
// ------------------------------

// 전진: 좌우 모터를 모두 전진 방향으로 동작시킴
void forward() {
  analogWrite(ENA, speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  analogWrite(ENB, speed);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// 후진: 좌우 모터를 모두 후진 방향으로 동작시킴
void backward() {
  analogWrite(ENA, speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  analogWrite(ENB, speed);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

// 좌회전: 왼쪽 모터는 후진, 오른쪽 모터는 전진하여 회전
void left() {
  analogWrite(ENA, speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  analogWrite(ENB, speed);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

// 우회전: 왼쪽 모터는 전진, 오른쪽 모터는 후진하여 회전
void right() {
  analogWrite(ENA, speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  analogWrite(ENB, speed);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// 차량 정지: 두 모터의 PWM 값을 0으로 설정하여 모터 정지
void stopCar() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
