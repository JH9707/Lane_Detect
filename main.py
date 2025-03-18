"""
프로젝트: 자율 주행 차선 검출 및 차선이탈 방지 시스템
설명: 
  이 프로그램은 PiCamera를 사용하여 실시간으로 영상을 캡처하고,
  OpenCV를 이용해 노란색 차선(또는 관심 색상)을 검출합니다.
  Hough 변환을 통해 차선의 각도를 계산한 후, 해당 각도를 Arduino에 전송하여 차량 제어를 수행합니다.
  
필요한 라이브러리:
  - OpenCV (cv2)
  - NumPy (numpy)
  - PySerial (serial)
  - Picamera2 (picamera2)
  - threading (멀티스레딩)
  
사용 방법:
  자동:
    - 프로그램 실행 후, 차선의 각도를 계산 후, 아두이노에 전송하여 차량을 제어합니다.

  수동:
    - 프로그램 실행 후, 터미널에서 키보드 입력(w, s, a, d, x, +, -, p, q)을 통해 제어합니다.
    - 'p' 입력 시 자율 주행과 수동 제어를 전환합니다.
"""

import cv2
import numpy as np
import serial
from picamera2 import Picamera2
import threading

# ===============================
# Serial Port 초기화 및 연결
# ===============================
try:
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    print("Serial connection established with Arduino.")
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    exit()

# ===============================
# 색상 필터 설정 (노란색 범위)
# 해당 필터의 경우 실제 차선 색상 정보를 토대로 수정 필요
# ===============================
h_min, h_max = 20, 36
s_min, s_max = 50, 255
v_min, v_max = 185, 255
yellow_min = (h_min, s_min, v_min)
yellow_max = (h_max, s_max, v_max)

# ===============================
# Hough 변환 파라미터 설정
# ===============================
rho = 2
theta = np.pi / 180  # 1도 단위
threshold = 15
min_line_len = 10
max_line_gap = 20

# ===============================
# 전역 변수 초기화
# ===============================
key_input_flag = False   # 키 입력 감지 플래그
command = ''             # 입력된 커맨드 저장
paused = False           # 자율 주행 정지/재개 플래그

# ===============================
# 함수 정의: 이미지 색상 필터링
# ===============================
def filter_colors(image):
    """
    입력 이미지에서 노란색 영역만 필터링
    매개변수:
      image: BGR 색상공간의 이미지
    반환:
      노란색 영역만 남긴 이미지
    """
    image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    yellow_mask = cv2.inRange(image_hsv, yellow_min, yellow_max)
    yellow_image = cv2.bitwise_and(image, image, mask=yellow_mask)
    return yellow_image

# ===============================
# 함수 정의: 관심 영역(ROI) 적용
# ===============================
def region_of_interest(image):
    """
    이미지에서 관심 영역(ROI)을 설정하여 불필요한 부분을 마스킹
    매개변수:
      image: 입력 이미지 (grayscale 혹은 edge map)
    반환:
      ROI가 적용된 이미지
    """
    height, width = image.shape[:2]
    vertices = np.array([
        [width * 0.1, height],
        [width * 0.4, int(height * 0.5)],
        [width * 0.6, int(height * 0.5)],
        [width * 0.9, height]
    ], dtype=np.int32)
    vertices = vertices.reshape((-1, 1, 2))
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, [vertices], 255)
    return cv2.bitwise_and(image, mask)

# ===============================
# 함수 정의: 선들의 평균 각도 계산
# ===============================
def calculate_angle(lines):
    """
    검출된 선들의 평균 기울기를 계산하여 각도로 반환
    매개변수:
      lines: HoughLinesP 결과로 나온 선 리스트
    반환:
      평균 각도 (단위: 도)
    """
    if lines is None or len(lines) == 0:
        return 0  # 선이 없으면 0도 반환

    angle_sum = 0
    for line in lines:
        x1, y1, x2, y2 = line[0]
        angle_sum += np.arctan2(y2 - y1, x2 - x1)
    
    angle_avg = angle_sum / len(lines)
    angle_deg = np.degrees(angle_avg)  # 라디안 -> 도 변환
    return angle_deg

# ===============================
# 함수 정의: 계산된 각도를 Arduino로 전송
# ===============================
def send_angle_to_arduino(angle):
    """
    계산된 각도를 Arduino로 전송 (앞에 'A'를 붙여 전송)
    매개변수:
      angle: 계산된 각도 (도)
    """
    ser.write(f"A{int(angle)}\n".encode())

# ===============================
# 함수 정의: 일시정지/재개 명령 전송
# ===============================
def send_pause_resume_to_arduino():
    """
    Arduino로 일시정지/재개 명령('p')를 전송
    """
    ser.write(b'p')

# ===============================
# 함수 정의: 사용자 키 입력 처리 스레드
# ===============================
def handle_key_input():
    """
    터미널에서 키 입력을 받아 차량 제어나 프로그램 종료를 처리
    w, s, a, d, x, +, - : 차량 제어 명령 전송
    p : 일시정지/재개 전환 (추가 명령 전송)
    q : 프로그램 종료
    """
    global key_input_flag, command, paused
    while True:
        user_input = input("Enter command (w, s, a, d, x, +, -) or press 'p' to pause/unpause, 'q' to quit: ").strip().lower()

        if user_input == 'q':
            print("Exiting program...")
            key_input_flag = True
            command = 'q'
            break
        elif user_input in ['w', 's', 'a', 'd', 'x', '+', '-']:
            key_input_flag = True
            command = user_input
        elif user_input == 'p':
            paused = not paused
            key_input_flag = True
            send_pause_resume_to_arduino()
            if paused:
                print("Car movement paused")
            else:
                print("Car movement resumed")

# ===============================
# 메인 프로그램: 카메라 및 제어 로직
# ===============================
try:
    # PiCamera 초기화 및 시작
    picam2 = Picamera2()
    picam2.start()

    # 키 입력 처리를 위한 스레드 시작 (백그라운드 실행)
    key_input_thread = threading.Thread(target=handle_key_input, daemon=True)
    key_input_thread.start()

    while True:
        # 사용자 키 입력이 감지되면 처리
        if key_input_flag:
            if command == 'q':
                break  # 'q' 입력 시 프로그램 종료
            elif command == 'p':
                # 'p' 명령은 handle_key_input()에서 이미 처리
                pass
            else:
                # 입력된 명령을 Arduino로 전송
                ser.write(command.encode())
                print(f"Sent command: {command}")
            key_input_flag = False  # 플래그 리셋

        # 일시정지 상태면 자율 주행 부분 스킵
        if paused:
            continue

        # ------------------------------
        # 자율 주행: 이미지 캡처 및 처리
        # ------------------------------
        frame = picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame = cv2.rotate(frame, cv2.ROTATE_180)

        # 차선 검출을 위한 이미지 전처리
        filtered_frame = filter_colors(frame)
        gray_frame = cv2.cvtColor(filtered_frame, cv2.COLOR_BGR2GRAY)
        gray_frame = cv2.GaussianBlur(gray_frame, (5, 5), 0)
        _, binary_frame = cv2.threshold(gray_frame, 0, 255, cv2.THRESH_OTSU + cv2.THRESH_BINARY)
        edges = cv2.Canny(binary_frame, 50, 150)
        roi = region_of_interest(edges)

        # HoughLinesP를 이용하여 선 검출
        lines = cv2.HoughLinesP(roi, rho, theta, threshold,
                                np.array([]),
                                minLineLength=min_line_len,
                                maxLineGap=max_line_gap)

        # 검출된 선들을 이용해 평균 각도 계산
        angle = calculate_angle(lines)

        # 계산된 각도를 Arduino로 전송
        send_angle_to_arduino(angle)

        # 검출된 선을 화면에 표시
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # 결과 영상 출력
        cv2.imshow('Lane Detection', frame)

        # OpenCV 창에서 'q' 키 입력 시 프로그램 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Program terminated by KeyboardInterrupt.")
finally:
    # 자원 정리: 카메라와 시리얼 포트, 모든 OpenCV 창 종료
    picam2.stop()
    picam2.close()
    cv2.destroyAllWindows()
    ser.close()
