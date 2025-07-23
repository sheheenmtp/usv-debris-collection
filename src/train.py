from ultralytics import YOLO

# Load a more powerful model (YOLOv8s instead of YOLOv8n)
model = YOLO("yolov8s.pt")

# Train the model with optimized parameters
model.train(
    data="/Users/sheheenmtp/Desktop/BTech/USV/YOLO/plastics/data.yaml",
    epochs=200,         # More epochs for better learning
    imgsz=960,         # Higher resolution improves accuracy
    batch=16,          # Increase batch size (adjust for RAM)
    workers=4,         # Use more CPU workers for data loading
    device='cpu',      # Change to '0' if you have a GPU
    optimizer='Adam',  # Adam optimizer for faster convergence
    lr0=1e-3,          # Initial learning rate
    cos_lr=True,       # Cosine learning rate scheduler
    weight_decay=0.00005,  # Reduce overfitting
    patience=20,       # Prevent stopping too early
    augment=True,      # Enable more data augmentation
    mosaic=1,          # Advanced augmentation
    conf=0.4,          # Confidence threshold tuning
    iou=0.45,          # IoU threshold for overlapping boxes
    project="PlasticBottleDetection",
    name="YOLOv8_Improved_Training"
)