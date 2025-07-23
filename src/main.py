import cv2
import numpy as np
from ultralytics import YOLO
import cvzone
import math
import time
import imutils
import urllib.request
import websocket

# Constants
TOLERANCE_VALUE = 8  # Center tolerance in pixels
FORWARD_DISTANCE_THRESHOLD = 15  # Move forward if distance is less than this (in cm)
pixels_per_cm = 10  # Scaling factor for distance calculation
prev_frame_time = 0

# Load YOLO Model
model = YOLO("/Users/sheheenmtp/Desktop/BTech/USV/YOLO/plastics/PlasticBottleDetection/YOLOv8_Improved_Training/weights/best.pt")

# Object Classes
classNames = ["plastic"]

# Initialize WebSocket connection
try:
    ws = websocket.create_connection("ws://192.168.47.84:8080")
    print("Connected to WebSocket server")
    websocket_connected = True
except Exception as e:
    print("WebSocket Connection Failed:", e)
    websocket_connected = False

# Mobile camera feed URL
url = "http://192.168.47.227:8080/shot.jpg"

def calculate_distance(point1, point2):
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

while True:
    # Capture frame from mobile camera
    try:
        imgPath = urllib.request.urlopen(url)
        imgNp = np.array(bytearray(imgPath.read()), dtype=np.uint8)
        img = cv2.imdecode(imgNp, -1)
        cam = imutils.resize(img, width=1080)
    except Exception as e:
        print("Error capturing image:", e)
        continue

    result = model.predict(source=cam, imgsz=416, conf=0.5)

    # Define frame center
    frame_center = (cam.shape[1] // 2, cam.shape[0] // 2)
    cv2.line(cam, (0, frame_center[1]), (cam.shape[1], frame_center[1]), (0, 0, 255), 2)
    cv2.line(cam, (frame_center[0], 0), (frame_center[0], cam.shape[0]), (0, 0, 255), 2)

    nearest_distance = float('inf')
    nearest_center = None

    for r in result:
        for box in r.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            x_center = (x1 + x2) // 2
            y_center = (y1 + y2) // 2

            # Calculate Distance
            distance = calculate_distance(frame_center, (x_center, y_center))
            distance_cm = round(distance / pixels_per_cm)

            # Track Nearest Object
            if distance_cm < nearest_distance:
                nearest_distance = distance_cm
                nearest_center = (x_center, y_center)

            # Draw Bounding Box and Labels
            cv2.rectangle(cam, (x1, y1), (x2, y2), (255, 0, 255), 5)
            cvzone.putTextRect(cam,
                               f'Plastic Detected - Distance: {distance_cm} cm',
                               (max(0, x1), max(35, y1)), scale=2, thickness=3, offset=10)

    # Determine Movement Direction
    movement_direction = "STOP"
    if nearest_center:
        cv2.circle(cam, nearest_center, 10, (0, 0, 255), -1)
        deviation_x = nearest_center[0] - frame_center[0]

        if nearest_distance < FORWARD_DISTANCE_THRESHOLD:
            movement_direction = "FORWARD"
        elif abs(deviation_x) > TOLERANCE_VALUE:
            movement_direction = "RIGHT" if deviation_x > 0 else "LEFT"

        cv2.putText(cam, f'Direction: {movement_direction}', (50, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Send movement command via WebSocket if connected
        if websocket_connected:
            try:
                ws.send(movement_direction)
            except Exception as e:
                print("Error sending WebSocket message:", e)
                websocket_connected = False

    # FPS Calculation
    new_frame_time = time.time()
    fps = 1 / (new_frame_time - prev_frame_time) if (new_frame_time - prev_frame_time) > 0 else 0
    prev_frame_time = new_frame_time
    print("FPS:", int(fps))

    # Show Output
    cv2.imshow("Debris Collector", cam)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Cleanup
if websocket_connected:
    ws.close()
cv2.destroyAllWindows()