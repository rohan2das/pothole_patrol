# Quick Start Guide

## 🚀 Getting Started in 5 Minutes

### Step 1: Install Dependencies

**On Laptop (Server):**
```bash
pip install flask flask-cors requests
```

**On Jetson Nano (Drone Client):**
```bash
pip install dronekit pymavlink opencv-python numpy ultralytics torch torchvision requests
```

### Step 2: Start the Server

**On your laptop:**
```bash
python server.py
```

You should see:
```
Starting GCS Server...
Access the web UI at: http://localhost:8000
 * Running on http://0.0.0.0:8000
```

### Step 3: Configure Drone Client

**Edit `drone_client_pothole.py` and update:**
- `GCS_SERVER_URL`: Your laptop's IP address (e.g., `http://192.168.1.100:8000`)
- `DRONEKIT_CONNECTION_STRING`: Your serial port (e.g., `/dev/ttyUSB0`)
- `CAMERA_INDEX`: Your camera index (usually `0` or `1`)

### Step 4: Start Drone Client

**On Jetson Nano:**
```bash
python drone_client_pothole.py --gcs-url http://YOUR_LAPTOP_IP:8000
```

Replace `YOUR_LAPTOP_IP` with your actual laptop IP.

### Step 5: Upload KML and Start Mission

1. Open browser: `http://localhost:8000`
2. Upload `sample_route.kml` (or your own KML file)
3. Click "Start Mission"
4. Watch the map for real-time detections!

## 📝 Testing Without Drone (Demo Mode)

You can test the entire system without a drone using the demo client:

1. **Start the server** (Terminal 1):
   ```bash
   python server.py
   ```

2. **Start the demo drone client** (Terminal 2):
   ```bash
   python demo_drone_client.py
   ```

3. **In the web UI:**
   - Upload a KML file (click "Upload")
   - Click "Process" to extract waypoints
   - Click "Start Mission"
   - The demo client will simulate the mission and send detections

4. **Customize detections:**
   - Edit `demo_detections.json` to add your own lat/lon coordinates
   - The demo client will use these detections when simulating the mission

The demo client will:
- Register with the server
- Wait for missions
- Simulate waypoint following
- Send detection logs from `demo_detections.json`
- Show anomalies on the map in real-time

## 🔧 Common Issues

**Server won't start:**
- Check if port 8000 is available
- Try: `lsof -i :8000` to see what's using it

**Drone client can't connect:**
- Verify laptop and Jetson are on same network
- Check firewall settings
- Ping laptop from Jetson: `ping YOUR_LAPTOP_IP`

**No detections:**
- YOLO model needs to be trained for potholes, or
- Adjust detection thresholds in code
- Check camera is working: `ls /dev/video*`

## 📚 Next Steps

- Train a custom YOLO model for better pothole detection
- Adjust severity thresholds based on your use case
- Add more features to the web UI
- Integrate with mapping services

Happy hacking! 🎉

