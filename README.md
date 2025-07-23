# Unmanned Surface Vehicle (USV) for Efficient Debris Collection

This project presents a specialized Unmanned Surface Vehicle (USV) designed for collecting plastic debris in confined water bodies like lakes, ponds, and rivers. The vehicle combines real-time object detection using YOLOv8 and autonomous navigation via ESP32-WROOM32, operating in both remote and autonomous modes.

## 🚀 Features

- Catamaran-inspired PVC hull for buoyancy and stability
- Remote control via Dabble (Bluetooth)
- Autonomous navigation using computer vision (YOLOv8)
- Real-time video feed using IP Webcam
- WebSocket-based communication to ESP32 for control
- Debris collection tray integrated between hulls

## 📦 Project Structure

```
usv-debris-collection/
├── README.md
├── LICENSE
├── .gitignore
├── requirements.txt
├── models/
├── src/
│   ├── main.py
│   ├── detection.py
│   ├── control.py
│   └── utils.py
├── hardware/
│   ├── esp32.ino
│   ├── block-diagram.png
│   ├── pin-diagram.png
│   └── circuit-diagram.png
├── docs/
└── data/
```

## 🛠️ Requirements

- Python 3.10+
- PyTorch
- Ultralytics (YOLOv8)
- OpenCV
- WebSocket (asyncio, websockets)
- ESP32 with Arduino IDE

## 🧠 How It Works

- Remote Mode: ESP32 receives commands via Bluetooth from Dabble App.
- Autonomous Mode: YOLOv8 detects plastic in video stream, calculates direction and sends movement commands via WebSocket.

## 📷 Hardware

- ESP32-WROOM-32
- IP Webcam (for live video stream)
- L298N Motor Driver
- 12V DC motors
- PVC frame catamaran design

## 🔧 Setup

```bash
git clone https://github.com/your-username/usv-debris-collection.git
cd usv-debris-collection
pip install -r requirements.txt
python src/main.py
```

## 📜 License

This project is licensed under the MIT License.
