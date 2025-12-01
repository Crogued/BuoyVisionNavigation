import cv2
import numpy as np
import serial
import time
from picamera2 import Picamera2

# Camera initialization
camera = Picamera2()
camera.configure(camera.create_preview_configuration(main={"format": 'XRGB8888', "size": (1280, 720)}))
camera.start()

# HSV range for red
lower_red1 = np.array([0, 150, 100])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([150, 110, 0])
upper_red2 = np.array([179, 255, 255])

# Font for text
font = cv2.FONT_HERSHEY_SIMPLEX

# Serial connection
serial_port = '/dev/ttyAMA0'
baudrate = 9600

try:
    arduino_serial = serial.Serial(serial_port, baudrate, timeout=1)
    time.sleep(2)
except serial.SerialException:
    print(f"Error opening serial port {serial_port}")
    arduino_serial = None

def calculate_circularity(contour):
    perimeter = cv2.arcLength(contour, True)
    area = cv2.contourArea(contour)
    if perimeter == 0:
        return 0
    circularity = 4 * np.pi * (area / (perimeter ** 2))
    return circularity

def detect_objects(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    objects = []

    for i, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if area < 600:  # Ignore noise
            continue

        M = cv2.moments(contour)
        if M["m00"] == 0:
            continue
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        circ = calculate_circularity(contour)

        objects.append({
            "name": f"Object {i+1}",
            "centroid": (cx, cy),
            "circularity": circ,
            "contour": contour,
            "area" : area
        })

    return objects

def draw_objects(frame, objects):
    for obj in objects:
        contour = obj["contour"]
        cx, cy = obj["centroid"]
        circ = obj["circularity"]
        name = obj["name"]
        area = obj["area"]
        text = f"{name}: ({cx},{cy}) C={circ:.2f} Ar={area:.0f}"

        # Green contour
        cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)

        # Blue dot at centroid
        cv2.circle(frame, (cx, cy), 4, (255, 0, 0), -1)

        # Text with black border for better readability
        cv2.putText(frame, text, (cx + 10, cy),
                    font, 0.9, (0, 0, 0), 6, cv2.LINE_AA)  # black shadow
        cv2.putText(frame, text, (cx + 10, cy),
                    font, 0.9, (255, 255, 255), 2, cv2.LINE_AA)  # white text on top

def send_serial(objects):
    for obj in objects:
        circ = obj["circularity"]
        if 0.75 <= circ <= 1:  # <<< ADJUSTABLE RANGE FOR BUOY
            cx, cy = obj["centroid"]
            message = f"{cx},{cy}\n"
            print(f"Sending: {message.strip()}")
            if arduino_serial and arduino_serial.is_open:
                arduino_serial.write(message.encode())

try:
    print("Press 'q' to exit.")

    window_name = "Red Object Detection"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 960, 540)

    while True:
        frame = camera.capture_array()
        objects = detect_objects(frame)
        draw_objects(frame, objects)
        send_serial(objects)

        cv2.imshow(window_name, frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    camera.close()
    cv2.destroyAllWindows()
    if arduino_serial and arduino_serial.is_open:
        arduino_serial.close()
