"""
Demo Drone Client - Simulates drone behavior for testing without actual hardware.
Run this on your laptop in a separate terminal to test the system.
"""
import json
import time
import requests
import threading
import datetime
import argparse
from pathlib import Path

# ===================== CONFIGURATION =====================
GCS_SERVER_URL = "http://localhost:8000"  # Change if server is on different machine
DRONE_ID = "demo_drone_01"
HEARTBEAT_INTERVAL = 5  # seconds
MISSION_SIMULATION_DELAY = 2  # seconds between waypoint simulation
# =========================================================

# Demo detection log file - you can edit this JSON file with your own lat/lon entries
DETECTION_LOG_FILE = "demo_detections.json"

def create_demo_detection_file():
    """Create a sample detection log file if it doesn't exist."""
    if not Path(DETECTION_LOG_FILE).exists():
        sample_detections = [
            {
                "timestamp": datetime.datetime.now().isoformat(),
                "frame_id": 1,
                "object": "pothole",
                "severity": "minor",
                "confidence": 0.75,
                "bbox_area": 6500,
                "latitude": 20.5937,
                "longitude": 78.9629,
                "altitude_m": 15.0
            },
            {
                "timestamp": datetime.datetime.now().isoformat(),
                "frame_id": 45,
                "object": "pothole",
                "severity": "major",
                "confidence": 0.88,
                "bbox_area": 18000,
                "latitude": 20.5940,
                "longitude": 78.9635,
                "altitude_m": 15.0
            },
            {
                "timestamp": datetime.datetime.now().isoformat(),
                "frame_id": 120,
                "object": "pothole",
                "severity": "minor",
                "confidence": 0.72,
                "bbox_area": 5500,
                "latitude": 20.5943,
                "longitude": 78.9641,
                "altitude_m": 15.0
            }
        ]
        
        with open(DETECTION_LOG_FILE, 'w') as f:
            json.dump(sample_detections, f, indent=4)
        
        print(f"✓ Created sample detection file: {DETECTION_LOG_FILE}")
        print("  You can edit this file to add your own lat/lon coordinates")
        return sample_detections
    else:
        with open(DETECTION_LOG_FILE, 'r') as f:
            return json.load(f)

def register_with_gcs():
    """Register demo drone with GCS server."""
    try:
        response = requests.post(
            f"{GCS_SERVER_URL}/drone-register",
            json={'drone_id': DRONE_ID},
            timeout=5
        )
        if response.status_code == 200:
            print(f"✓ Registered with GCS: {response.json()}")
            return True
        else:
            print(f"✗ Registration failed: {response.status_code}")
            return False
    except Exception as e:
        print(f"✗ Error registering with GCS: {e}")
        print(f"  Make sure the server is running at {GCS_SERVER_URL}")
        return False

def send_heartbeat(current_waypoint=0, mission_status='idle'):
    """Send periodic heartbeat to GCS."""
    try:
        response = requests.post(
            f"{GCS_SERVER_URL}/drone-heartbeat",
            json={
                'drone_id': DRONE_ID,
                'current_waypoint': current_waypoint,
                'mission_status': mission_status
            },
            timeout=2
        )
        if response.status_code == 200:
            return True
        else:
            print(f"Heartbeat failed with status {response.status_code}")
            return False
    except Exception as e:
        print(f"Heartbeat error: {e}")
        return False

def get_mission_from_gcs():
    """Get mission waypoints from GCS."""
    try:
        response = requests.get(f"{GCS_SERVER_URL}/get-mission", timeout=5)
        if response.status_code != 200:
            return None
        
        mission_info = response.json()
        waypoints_url = mission_info.get('waypoints_url') or mission_info.get('waypoints_file')
        
        if not waypoints_url:
            return None
        
        if waypoints_url.startswith('/'):
            waypoints_url = f"{GCS_SERVER_URL}{waypoints_url}"
        else:
            waypoints_url = f"{GCS_SERVER_URL}/{waypoints_url}"
        
        print(f"Downloading waypoints file from {waypoints_url}...")
        file_response = requests.get(waypoints_url, timeout=10)
        
        if file_response.status_code != 200:
            print(f"✗ Failed to download waypoints file: {file_response.status_code}")
            return None
        
        mission_data = file_response.json()
        print(f"✓ Mission loaded: {len(mission_data['waypoints'])} waypoints")
        return mission_data
    
    except Exception as e:
        print(f"✗ Error getting mission: {e}")
        return None

def check_start_command():
    """Check if GCS has sent start command."""
    try:
        response = requests.get(f"{GCS_SERVER_URL}/drone-status", timeout=2)
        if response.status_code == 200:
            status = response.json()
            return status.get('mission_running', False)
        return False
    except:
        return False

def check_stop_command():
    """Check if GCS has sent stop command."""
    try:
        response = requests.get(f"{GCS_SERVER_URL}/get-mission", timeout=2)
        if response.status_code == 200:
            mission = response.json()
            return mission.get('stop_requested', False)
        return False
    except:
        return False

def send_detection_log(mission_id, detection_log):
    """Send detection log to GCS after mission completion."""
    try:
        response = requests.post(
            f"{GCS_SERVER_URL}/mission-complete",
            json={
                'drone_id': DRONE_ID,
                'mission_id': mission_id,
                'detection_log': detection_log
            },
            timeout=10
        )
        
        if response.status_code == 200:
            print(f"✓ Detection log sent to GCS: {len(detection_log)} anomalies")
            return True
        else:
            print(f"✗ Failed to send log: {response.status_code}")
            return False
    except Exception as e:
        print(f"✗ Error sending detection log: {e}")
        return False

def simulate_mission(waypoints, detection_log):
    """Simulate mission execution by going through waypoints."""
    print("\n" + "=" * 60)
    print("Starting Mission Simulation")
    print("=" * 60)
    
    print(f"Mission has {len(waypoints)} waypoints")
    print(f"Simulating {len(detection_log)} detections")
    print("\nSimulating flight...")
    
    for i, waypoint in enumerate(waypoints):
        if check_stop_command():
            print("\n⚠ STOP COMMAND RECEIVED!")
            return False
        
        print(f"  Waypoint {i+1}/{len(waypoints)}: [{waypoint[0]:.6f}, {waypoint[1]:.6f}]")
        time.sleep(MISSION_SIMULATION_DELAY)
    
    print("\n✓ Mission simulation complete!")
    print("=" * 60)
    return True

def heartbeat_loop():
    """Background thread for sending heartbeats."""
    current_waypoint = 0
    mission_status = 'idle'
    
    while True:
        try:
            send_heartbeat(current_waypoint, mission_status)
            time.sleep(HEARTBEAT_INTERVAL)
        except Exception as e:
            print(f"Heartbeat loop error: {e}")
            time.sleep(HEARTBEAT_INTERVAL)

def main():
    global GCS_SERVER_URL
    global DETECTION_LOG_FILE
    
    parser = argparse.ArgumentParser(description="Demo Drone Client - Simulates drone for testing")
    parser.add_argument("--gcs-url", default=GCS_SERVER_URL, help="GCS server URL")
    parser.add_argument("--detections", default=DETECTION_LOG_FILE, help="Path to detection log JSON file")
    args = parser.parse_args()
    
    GCS_SERVER_URL = args.gcs_url
    DETECTION_LOG_FILE = args.detections
    
    print("=" * 60)
    print("Demo Drone Client Starting...")
    print("=" * 60)
    print(f"GCS Server: {GCS_SERVER_URL}")
    print(f"Drone ID: {DRONE_ID}")
    print(f"Detection Log: {DETECTION_LOG_FILE}")
    print("=" * 60)
    
    # Load or create detection log
    print("\nLoading detection log...")
    detection_log = create_demo_detection_file()
    print(f"✓ Loaded {len(detection_log)} detections from {DETECTION_LOG_FILE}")
    print("  Edit this file to add your own lat/lon coordinates")
    
    # Register with GCS
    print("\nRegistering with GCS...")
    while not register_with_gcs():
        print("Retrying registration in 5 seconds...")
        time.sleep(5)
    
    # Start heartbeat thread
    print("\nStarting heartbeat thread...")
    heartbeat_thread = threading.Thread(target=heartbeat_loop, daemon=True)
    heartbeat_thread.start()
    print("✓ Heartbeat thread started")
    
    # Main loop
    print("\nWaiting for mission from GCS...")
    print("(Upload and process a KML file in the web UI, then click 'Start Mission')")
    print("-" * 60)
    
    mission_loaded = False
    mission_running = False
    
    while True:
        try:
            # Check if mission is loaded
            if not mission_loaded:
                mission_data = get_mission_from_gcs()
                if mission_data:
                    mission_loaded = True
                    mission_id = mission_data['mission_id']
                    waypoints = mission_data['waypoints']
                    print(f"\n✓ Mission loaded: {len(waypoints)} waypoints")
                    print("  Waiting for start command...")
            
            # Check if start command received
            if mission_loaded and not mission_running:
                if check_start_command():
                    print("\n" + "=" * 60)
                    print("START COMMAND RECEIVED!")
                    print("=" * 60)
                    mission_running = True
                    
                    # Simulate mission
                    success = simulate_mission(waypoints, detection_log)
                    
                    if success:
                        # Send detection log
                        print("\nSending detection log to GCS...")
                        send_detection_log(mission_id, detection_log)
                        
                        print("\n✓ Mission completed successfully!")
                        print("  Check the web UI to see the anomalies on the map")
                    
                    mission_running = False
                    mission_loaded = False
                    print("\nWaiting for new mission...")
                    print("-" * 60)
            
            # Check for stop command
            if mission_running:
                if check_stop_command():
                    print("\n⚠ STOP COMMAND RECEIVED!")
                    mission_running = False
                    mission_loaded = False
            
            time.sleep(1)
            
        except KeyboardInterrupt:
            print("\n\n⚠ Keyboard Interrupt received! Shutting down...")
            break
        except Exception as e:
            print(f"Error in main loop: {e}")
            time.sleep(5)
    
    print("\n" + "=" * 60)
    print("Demo Drone Client Stopped")
    print("=" * 60)

if __name__ == "__main__":
    main()

