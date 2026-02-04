import time
from enum import Enum
import cv2
import numpy as np
import serial

# ================= SERIAL SETTINGS =================
SERIAL_PORT = "COM3"
BAUD = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.1)
    time.sleep(2)  
    print(f"Connected to Arduino on {SERIAL_PORT}")
except Exception as e:
    print(f"WARNING: Serial connection failed ({e}). Running in Camera-only mode.")
    ser = None

# ================= CAMERA SETTINGS =================
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW) 
kernel = np.ones((5, 5), np.uint8)
AREA_THRESHOLD = 300
CONFIRM_FRAMES = 5  

COLORS = {
    "RED":   ([136, 87, 111], [180, 255, 255]),
    "GREEN": ([25, 52, 72],   [102, 255, 255]),
    "BLUE":  ([94, 80, 2],    [120, 255, 255]),
}

class State(Enum):
    IDLE = 0
    DETECTED = 1
    ACTION = 2

state = State.IDLE
current_color = "NONE"
frame_count = 0
print("Starting Vision System... Press 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    best_color_name = "NONE"
    max_area_found = 0
    best_contour = None

    # ======= COLOR DETECTION =======
    for name, (low, high) in COLORS.items():
        mask = cv2.inRange(hsv, np.array(low), np.array(high))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if cnts:
            c = max(cnts, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if area > AREA_THRESHOLD and area > max_area_found:
                max_area_found = area
                best_color_name = name
                best_contour = c

    detected_color = best_color_name

    if best_contour is not None:
        x, y, w, h = cv2.boundingRect(best_contour)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(frame, f"{best_color_name}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # ======= STATE LOGIC =======
    if state == State.IDLE:
        if detected_color != "NONE":
            current_color = detected_color
            frame_count = 1
            state = State.DETECTED

    elif state == State.DETECTED:
        if detected_color == current_color:
            frame_count += 1
            if frame_count >= CONFIRM_FRAMES:
                if ser:
                    ser.reset_output_buffer() 
                    ser.write(f"{current_color}\n".encode())
                    ser.flush()  # Ensure data is sent immediately
                    time.sleep(0.05)  # Small delay to ensure transmission
                print(f"COMMAND SENT: {current_color}")
                state = State.ACTION
        else:
            state = State.IDLE

    elif state == State.ACTION:
        if detected_color != current_color:
            if ser:
                ser.write(b"NONE\n")
                ser.flush()
            print("RESETTING...")
            state = State.IDLE

    # ======= DISPLAY =======
    cv2.putText(frame, f"State: {state.name}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(frame, f"Color: {detected_color}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.imshow("Vision State Machine", frame)
    
    if cv2.waitKey(10) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
if ser:
    ser.close()
    print("Serial connection closed.")
