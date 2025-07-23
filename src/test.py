from ultralytics import YOLO
import cv2
import time
import threading
import queue

# Global variables
frame_queue = queue.Queue(maxsize=1)  # Only store latest frame
result_queue = queue.Queue(maxsize=1)  # Only store latest result
exit_event = threading.Event()


def capture_frames():
    """Thread function to capture frames continuously"""
    cap = cv2.VideoCapture(0)

    # Set lowest possible resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)  # Ultra-low resolution
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    while not exit_event.is_set():
        ret, frame = cap.read()
        if not ret:
            break

        # Always replace old frame with new one
        if frame_queue.full():
            try:
                frame_queue.get_nowait()
            except queue.Empty:
                pass
        frame_queue.put(frame)

    cap.release()


def process_frames():
    """Thread function to process frames with YOLO"""
    # Load model with optimized settings
    model = YOLO(
        "/Users/sheheenmtp/Desktop/BTech/USV/YOLO/plastics/PlasticBottleDetection/YOLOv8_Improved_Training/weights/best.pt")

    # Force model to use half precision (if GPU available)
    model.to("cuda" if model.device.type == "cuda" else "cpu")

    # Process every 4th frame
    frame_count = 0

    while not exit_event.is_set():
        if frame_queue.empty():
            time.sleep(0.01)  # Short sleep to prevent CPU hogging
            continue

        try:
            frame = frame_queue.get_nowait()
            frame_count += 1

            # Skip frames to reduce load
            if frame_count % 4 != 0:
                continue

            # Further reduce frame size for inference
            small_frame = cv2.resize(frame, (256, 256))

            # Run detection with minimal settings
            results = model.predict(
                small_frame,
                conf=0.7,
                iou=0.7,  # Higher IoU threshold for fewer overlapping boxes
                max_det=5,  # Only show top 5 detections
                verbose=False
            )

            # Update result queue
            if result_queue.full():
                try:
                    result_queue.get_nowait()
                except queue.Empty:
                    pass
            result_queue.put((frame, results[0]))

        except queue.Empty:
            continue


def main():
    # Start worker threads
    capture_thread = threading.Thread(target=capture_frames)
    process_thread = threading.Thread(target=process_frames)

    capture_thread.daemon = True
    process_thread.daemon = True

    capture_thread.start()
    process_thread.start()

    # Variables for FPS calculation
    fps_start_time = time.time()
    fps_counter = 0
    fps = 0

    while True:
        if result_queue.empty():
            # Display a blank frame with FPS when no results
            blank_frame = cv2.imread('/Users/sheheenmtp/Desktop/blank.jpg') if fps_counter == 0 else None
            if blank_frame is not None:
                blank_frame = cv2.resize(blank_frame, (320, 240))
                cv2.putText(blank_frame, "Initializing...", (50, 120),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.imshow("YOLOv8 Detection", blank_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            time.sleep(0.01)
            continue

        # Get latest results
        try:
            frame, result = result_queue.get_nowait()
            detections = result.boxes.data

            # Calculate FPS
            fps_counter += 1
            if fps_counter >= 10:
                fps = fps_counter / (time.time() - fps_start_time)
                fps_start_time = time.time()
                fps_counter = 0

            # Process detections
            if len(detections) > 0:
                for box in detections:
                    x1, y1, x2, y2, confidence, class_id = box.tolist()
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                    class_id = int(class_id)

                    # Minimal drawing
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 1)  # Thinner line

                    # Simpler label
                    cv2.putText(frame, f"{class_id}", (x1, y1 - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)  # Smaller text

            # Add minimal FPS display
            cv2.putText(frame, f"FPS: {fps:.1f}", (5, 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

            # Display frame
            cv2.imshow("YOLOv8 Detection", frame)

        except queue.Empty:
            continue

        # Exit on 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Signal threads to exit and wait for them
    exit_event.set()
    capture_thread.join(timeout=1.0)
    process_thread.join(timeout=1.0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()