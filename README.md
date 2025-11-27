# Road Surface Anomaly Detection System

AI-Driven Cyber-Physical System for detecting potholes and road anomalies using drones equipped with real-time YOLO-based computer vision.

[![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org/)
[![YOLO](https://img.shields.io/badge/YOLO-11n-red.svg)](https://github.com/ultralytics/ultralytics)
[![Flask](https://img.shields.io/badge/Flask-3.0+-green.svg)](https://flask.palletsprojects.com/)

## 🎯 Project Overview

This system enables autonomous drone missions to survey roads and detect potholes in real-time. The drone follows waypoints extracted from KML files, uses a custom-trained YOLO11n model for anomaly detection, and maps detected potholes with GPS coordinates on an interactive web interface.

### Key Features

- **Autonomous Mission Planning**: Upload KML files to define flight paths
- **Real-time Pothole Detection**: Custom-trained YOLO11n model for accurate detection
- **Severity Classification**: Automatically classifies potholes as minor or major
- **GPS Mapping**: Records exact location (lat/lon) of each detected anomaly
- **Interactive Web UI**: Visualize waypoints and anomalies on OpenStreetMap
- **Road Conditions Map**: Traffic-style visualization of road quality
- **Color-coded Visualization**: 
  - 🟢 Green: Good road
  - 🟡 Yellow: Minor pothole / Moderate road
  - 🔴 Red: Major pothole / Poor road

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

## 📊 Dataset

This project uses a custom dataset for training the pothole detection model:

**Dataset**: [Pothole Detection Dataset on Roboflow](https://universe.roboflow.com/rohan-das-c06cw/pothole-pq3lu-purng/dataset/1)

### Dataset Details
- **Source**: Roboflow Universe
- **Format**: YOLO format annotations
- **Classes**: Pothole
- **Use Case**: Road surface anomaly detection

The dataset contains annotated images of roads with potholes, suitable for training object detection models. The dataset is available on Roboflow Universe and can be downloaded for training or evaluation purposes.

## 🤖 Model Architecture

### YOLO11n (YOLOv11 Nano)

This project uses a **custom-trained YOLO11n** model for pothole detection. YOLO11n is the latest iteration of the YOLO (You Only Look Once) architecture, optimized for real-time object detection on edge devices.

#### Model Specifications

- **Architecture**: YOLO11n (Nano variant)
- **Input Size**: 640x640 pixels
- **Framework**: Ultralytics YOLO
- **Training**: Custom trained on pothole detection dataset
- **Model File**: `pothole_patrol.pt`

#### Model Architecture Details

YOLO11n is based on the YOLO architecture with the following key features:

1. **Backbone**: CSPDarknet-based feature extractor
2. **Neck**: PANet (Path Aggregation Network) for feature fusion
3. **Head**: Detection head with anchor-free design
4. **Optimization**: Optimized for edge devices (Jetson Nano)

#### Performance Characteristics

- **Speed**: Real-time inference on Jetson Nano (~30 FPS)
- **Accuracy**: Optimized for pothole detection in aerial imagery
- **Size**: Nano variant for efficient deployment on edge devices

#### Using the Pre-trained Model

The custom-trained model (`pothole_patrol.pt`) is included in this repository. To use it:

1. Ensure the model file is in the project root directory
2. The model path is configured in `drone_client_pothole.py`:
   ```python
   MODEL_PATH = "pothole_patrol.pt"
   ```
3. The model will be automatically loaded when running the drone client

#### Training Your Own Model

If you want to train a custom model using the same dataset:

1. **Download Dataset**: 
   - Visit the [Roboflow dataset page](https://universe.roboflow.com/rohan-das-c06cw/pothole-pq3lu-purng/dataset/1)
   - Download the dataset in YOLO format

2. **Train Model**:
   ```python
   from ultralytics import YOLO
   
   # Initialize YOLO11n model
   model = YOLO('yolo11n.pt')
   
   # Train on your dataset
   model.train(
       data='path/to/dataset.yaml',
       epochs=100,
       imgsz=640,
       batch=16,
       device=0  # Use GPU if available
   )
   
   # Save the trained model
   model.export(format='pt')
   ```

3. **Replace Model**: Replace `pothole_patrol.pt` with your trained model

## 💻 Code Implementation

### Project Structure

```
pothole_patrol/
├── server.py                 # Flask server with web UI
├── drone_client_pothole.py   # Jetson Nano client with YOLO11n
├── demo_drone_client.py      # Demo client for testing without drone
├── pothole_patrol.pt        # Custom-trained YOLO11n model
├── demo_detections.json      # Sample detection log for demo client
├── sample_route.kml          # Sample KML file for testing
├── templates/
│   ├── index.html           # Mission planning web interface
│   └── road_map.html        # Road conditions map view
├── requirements.txt          # Python dependencies
└── README.md                # This file
```

### Key Components

#### 1. Server (`server.py`)
- Flask-based web server
- KML file processing and waypoint extraction
- Mission management API
- Real-time anomaly data storage
- Road conditions aggregation

#### 2. Drone Client (`drone_client_pothole.py`)
- YOLO11n model loading and inference
- Real-time pothole detection
- GPS coordinate logging
- Mission execution with waypoint following
- Severity classification based on bounding box area

#### 3. Web Interface (`templates/`)
- Mission planning page with KML upload
- Interactive map visualization
- Road conditions map view
- Real-time anomaly display

## 🚀 Setup and Installation

### Prerequisites

**Server (Laptop)**
- Python 3.8+
- Flask and dependencies
- Web browser

**Drone Client (Jetson Nano)**
- Python 3.8+
- Jetson Nano with JetPack
- Camera connected
- Pixhawk/ArduPilot autopilot

### Installation Steps

#### 1. Clone Repository

```bash
git clone https://github.com/yourusername/pothole_patrol.git
cd pothole_patrol
```

#### 2. Install Server Dependencies

```bash
pip install -r requirements.txt
```

#### 3. Install Drone Client Dependencies (on Jetson Nano)

```bash
# Install PyTorch for Jetson (if not already installed)
# Follow NVIDIA's instructions for your JetPack version

pip install -r requirements.txt
```

#### 4. Model Setup

The project includes a custom-trained YOLO11n model (`pothole_patrol.pt`). No additional setup needed - it will be loaded automatically.

### Configuration

#### Server Configuration (`server.py`)

Update these settings if needed:
- `UPLOAD_FOLDER`: Where KML files are stored
- `MISSIONS_FOLDER`: Where mission JSON files are stored
- Default port: `8000`

#### Drone Client Configuration (`drone_client_pothole.py`)

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

# Model
MODEL_PATH = "pothole_patrol.pt"

# Detection Thresholds
SCORE_THRESHOLD = 0.5
MINOR_POTHOLE_AREA_THRESHOLD = 5000  # pixels^2
MAJOR_POTHOLE_AREA_THRESHOLD = 15000  # pixels^2

# Flight Parameters
TARGET_ALTITUDE = 15  # meters
TARGET_SPEED = 2.5  # m/s
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
2. Click "Upload" to select a KML file
3. Click "Process" to extract waypoints
4. The system will display waypoints on the map

### 4. Start Mission

1. Select a mission from the list
2. Click "Start Mission"
3. The drone will:
   - Take off to target altitude
   - Follow waypoints in sequence
   - Continuously detect potholes using YOLO11n
   - Record GPS coordinates of anomalies
   - Return to launch after completion

### 5. View Results

- Anomalies appear on the map in real-time
- Color coding shows severity (green/yellow/red)
- Click markers for detailed information
- Detection logs are saved in `run_<mission_id>/detection_log.json`
- View road conditions map at `/road-map`

### Testing Without Drone (Demo Mode)

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

## 🔍 API Endpoints

### Server API

- `GET /` - Web interface (mission planning)
- `GET /road-map` - Road conditions map view
- `POST /api/upload-kml` - Upload KML file
- `POST /api/process-kml` - Process uploaded KML file
- `GET /api/missions` - List all missions
- `GET /api/mission/<mission_id>` - Get mission details
- `POST /api/mission/<mission_id>/start` - Start mission
- `POST /api/mission/<mission_id>/stop` - Stop mission
- `GET /api/road-conditions` - Get aggregated road conditions

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
- **YOLO model not loading**: Ensure `pothole_patrol.pt` exists in project root
- **Drone not connecting**: Check serial port and baud rate

### Detection Issues

- **No detections**: Adjust `SCORE_THRESHOLD` or retrain model
- **False positives**: Retrain model or adjust area thresholds
- **Severity classification wrong**: Adjust `MINOR_POTHOLE_AREA_THRESHOLD` and `MAJOR_POTHOLE_AREA_THRESHOLD`

## 📝 Notes

- The system uses area-based severity classification. For production, consider training a model that directly classifies severity.
- GPS accuracy depends on your drone's GPS module.
- Detection performance depends on lighting, altitude, and camera quality.
- For production use, add proper error handling and safety checks.

## 📄 License

This project is licensed under the MIT License.

## 🙏 Acknowledgments

- **Dataset**: [Roboflow Universe - Pothole Detection Dataset](https://universe.roboflow.com/rohan-das-c06cw/pothole-pq3lu-purng/dataset/1)
- **YOLO Framework**: [Ultralytics YOLO](https://github.com/ultralytics/ultralytics)
- **Drone Control**: [DroneKit](https://github.com/dronekit/dronekit-python)
- **Mapping**: [Leaflet](https://leafletjs.com/) and [OpenStreetMap](https://www.openstreetmap.org/)

---

**Happy Flying! 🚁**
