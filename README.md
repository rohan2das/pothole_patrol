# Road Surface Anomaly Detection System

AI-Driven Cyber-Physical System for detecting potholes and road anomalies using drones equipped with real-time YOLO-based computer vision.

## 🎯 Project Overview

This system enables autonomous drone missions to survey roads and detect potholes in real-time. The drone follows waypoints extracted from KML files, uses YOLO for anomaly detection, and maps detected potholes with GPS coordinates on an interactive web interface.

### Features

- **KML Waypoint Processing**: Upload KML files to define flight paths
- **Real-time Detection**: YOLO-based pothole detection during flight
- **Severity Classification**: Automatically classifies potholes as minor or major
- **GPS Mapping**: Records exact location (lat/lon) of each detected anomaly
- **Interactive Web UI**: Visualize waypoints and anomalies on OpenStreetMap
- **Color-coded Visualization**: 
  - 🟢 Green: Good road
  - 🟡 Yellow: Minor pothole
  - 🔴 Red: Major pothole

## 🏗️ System Architecture

```
┌─────────────────┐
│   Laptop Server │  ← Web UI, KML Processing, Mission Management
│   (Flask)       │
└────────┬────────┘
         │ HTTP/API
         │
┌────────▼────────┐
│  Jetson Nano    │  ← YOLO Detection, Drone Control
│  (Companion PC) │
└────────┬────────┘
         │ MAVLink
         │
┌────────▼────────┐
│   Drone/Pixhawk │  ← Flight Control, GPS, Camera
└─────────────────┘
```

## 📋 Prerequisites

### Server (Laptop)
- Python 3.8+
- Flask and dependencies
- Web browser

### Drone Client (Jetson Nano)
- Python 3.8+
- Jetson Nano with JetPack
- Camera connected
- Pixhawk/ArduPilot autopilot
- YOLO model (can use pre-trained or custom trained)

## 🚀 Installation

### 1. Clone/Download Project

```bash
cd /home/rohan/Desktop/project/pothole_patrol
```

### 2. Install Server Dependencies

```bash
pip install -r requirements.txt
```

### 3. Install Drone Client Dependencies (on Jetson Nano)

```bash
# Install PyTorch for Jetson (if not already installed)
# Follow NVIDIA's instructions for your JetPack version

pip install -r requirements.txt
```

### 4. Download YOLO Model

The system uses YOLOv8 by default. You can:

**Option A: Use Pre-trained YOLOv8** (will auto-download on first run)
```bash
# Model will be downloaded automatically when you run the client
```

**Option B: Use Custom Trained Model**
1. Train a YOLO model on pothole dataset
2. Save as `pothole_detection_model.pt`
3. Update `MODEL_PATH` in `drone_client_pothole.py`

## 🔧 Configuration

### Server Configuration (`server.py`)

Update these settings if needed:
- `UPLOAD_FOLDER`: Where KML files are stored
- `MISSIONS_FOLDER`: Where mission JSON files are stored
- Default port: `8000`

### Drone Client Configuration (`drone_client_pothole.py`)

Update these settings for your setup:

```python
# Camera
CAMERA_INDEX = 0  # Change based on your camera
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720

# Drone Connection
DRONEKIT_CONNECTION_STRING = '/dev/ttyUSB0'  # Your serial port
DRONEKIT_BAUD = 57600  # Your baud rate

# GCS Server
GCS_SERVER_URL = "http://192.168.1.100:8000"  # Your laptop's IP
DRONE_ID = "jetson_drone_01"

# Flight Parameters
TARGET_ALTITUDE = 15  # meters
TARGET_SPEED = 2.5  # m/s

# Detection Thresholds
SCORE_THRESHOLD = 0.5
MINOR_POTHOLE_AREA_THRESHOLD = 5000  # pixels^2
MAJOR_POTHOLE_AREA_THRESHOLD = 15000  # pixels^2
```

## 🎮 Usage

### 1. Start the Server (on Laptop)

```bash
python server.py
```

The web interface will be available at: `http://localhost:8000`

### 2. Start the Drone Client (on Jetson Nano)

```bash
python drone_client_pothole.py --gcs-url http://YOUR_LAPTOP_IP:8000
```

Replace `YOUR_LAPTOP_IP` with your laptop's IP address (e.g., `192.168.1.100`).

### 3. Upload KML File

1. Open the web interface in your browser
2. Click or drag & drop a KML file
3. Click "Upload & Process"
4. The system will extract waypoints and display them on the map

### 4. Start Mission

1. Select a mission from the list
2. Click "Start Mission"
3. The drone will:
   - Take off to target altitude
   - Follow waypoints in sequence
   - Continuously detect potholes
   - Record GPS coordinates of anomalies
   - Return to launch after completion

### 5. View Results

- Anomalies appear on the map in real-time
- Color coding shows severity
- Click markers for detailed information
- Detection logs are saved in `run_<mission_id>/detection_log.json`

## 🧪 Testing Without Drone (Demo Mode)

For testing without actual drone hardware:

1. **Start server** (Terminal 1):
   ```bash
   python server.py
   ```

2. **Start demo client** (Terminal 2):
   ```bash
   python demo_drone_client.py
   ```

3. **Customize detections**: Edit `demo_detections.json` with your own lat/lon coordinates

4. **Use the web UI** to upload KML and start missions - the demo client will simulate everything!

The demo client mimics the real drone client behavior:
- Registers with server
- Sends heartbeats
- Downloads missions
- Simulates waypoint following
- Sends detection logs from `demo_detections.json`

## 📁 Project Structure

```
pothole_patrol/
├── server.py                 # Flask server with web UI
├── drone_client_pothole.py   # Jetson Nano client with YOLO
├── demo_drone_client.py      # Demo client for testing without drone
├── drone_client15112025v2.py # Original Coral Edge TPU client (reference)
├── demo_detections.json      # Sample detection log for demo client
├── sample_route.kml          # Sample KML file for testing
├── templates/
│   └── index.html           # Web interface
├── requirements.txt          # Python dependencies
├── README.md                # This file
├── QUICKSTART.md            # Quick start guide
├── uploads/                 # KML files (created automatically)
├── missions/                # Mission JSON files (created automatically)
└── run_<mission_id>/        # Mission output directories
    ├── waypoints.json
    ├── detection_log.json
    ├── raw_video.mp4
    └── detected_frames/     # Frames with detections
```

## 🎓 Training Custom YOLO Model

For better pothole detection accuracy, train a custom YOLO model:

1. **Collect Dataset**: Gather images of roads with/without potholes
2. **Label Data**: Use tools like LabelImg to annotate potholes
3. **Train Model**:
   ```bash
   from ultralytics import YOLO
   model = YOLO('yolov8n.pt')  # Start from pre-trained
   model.train(data='pothole_dataset.yaml', epochs=100)
   ```
4. **Use Custom Model**: Update `MODEL_PATH` in `drone_client_pothole.py`

## 🔍 API Endpoints

### Server API

- `GET /` - Web interface
- `POST /api/upload-kml` - Upload and process KML file
- `GET /api/missions` - List all missions
- `GET /api/mission/<mission_id>` - Get mission details
- `POST /api/mission/<mission_id>/start` - Start mission
- `POST /api/mission/<mission_id>/stop` - Stop mission
- `GET /api/anomalies/<mission_id>` - Get anomalies for mission

### Drone Client API (called by client)

- `POST /drone-register` - Register drone
- `POST /drone-heartbeat` - Send heartbeat
- `GET /get-mission` - Get mission waypoints
- `POST /mission-complete` - Submit detection log

## 🐛 Troubleshooting

### Server Issues

- **Port already in use**: Change port in `server.py` or kill existing process
- **KML parsing fails**: Ensure KML file has valid coordinates

### Drone Client Issues

- **Cannot connect to server**: Check IP address and firewall settings
- **Camera not working**: Verify camera index and permissions
- **YOLO model not loading**: Ensure model file exists or internet connection for download
- **Drone not connecting**: Check serial port and baud rate

### Detection Issues

- **No detections**: Adjust `SCORE_THRESHOLD` or train custom model
- **False positives**: Train custom model or adjust area thresholds
- **Severity classification wrong**: Adjust `MINOR_POTHOLE_AREA_THRESHOLD` and `MAJOR_POTHOLE_AREA_THRESHOLD`

## 📝 Notes

- The system uses area-based severity classification. For production, consider training a model that directly classifies severity.
- GPS accuracy depends on your drone's GPS module.
- Detection performance depends on lighting, altitude, and camera quality.
- For production use, add proper error handling and safety checks.

## 📄 License

This project is for hackathon/educational purposes.

## 🤝 Contributing

Feel free to improve the system:
- Better detection models
- Improved UI/UX
- Additional features
- Bug fixes

## 📧 Support

For issues or questions, check the code comments or create an issue in your repository.

---

**Happy Flying! 🚁**

