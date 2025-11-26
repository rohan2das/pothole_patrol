"""
Drone client for Jetson Nano with YOLO-based pothole detection.
Connects to GCS server, receives waypoints, executes mission, and sends anomaly logs back.
"""
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping

import cv2
import time
import os
import json
import math
import datetime
import argparse
import requests
import threading
import hashlib
import numpy as np
from pathlib import Path
from dronekit import connect, VehicleMode, LocationGlobalRelative

# Try to import YOLO (ultralytics)
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("Warning: ultralytics not available. Install with: pip install ultralytics")

# ===================== CONFIGURATION =====================
# YOLO Model configuration
MODEL_PATH = "yolov8n.pt"  # Can be yolov8n.pt, yolov8s.pt, yolov8m.pt, etc.
# For pothole detection, you may need a custom trained model
# MODEL_PATH = "pothole_detection_model.pt"  # Custom trained model

CAMERA_INDEX = 0  # Change based on your camera
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
SCORE_THRESHOLD = 0.5  # Confidence threshold for detections

# Pothole severity classification thresholds
MINOR_POTHOLE_AREA_THRESHOLD = 5000  # pixels^2 (adjust based on your needs)
MAJOR_POTHOLE_AREA_THRESHOLD = 15000  # pixels^2

DRONEKIT_CONNECTION_STRING = '/dev/ttyUSB0'  # Change based on your setup
DRONEKIT_BAUD = 57600

TARGET_ALTITUDE = 15  # meters
TARGET_SPEED = 2.5  # m/s

# GCS Server configuration
GCS_SERVER_URL = "http://192.168.1.100:8000"  # Update with your GCS IP
DRONE_ID = "jetson_drone_01"
HEARTBEAT_INTERVAL = 5  # seconds
# =========================================================

# Mission state
MISSION_STATE = {
    'waypoints': [],
    'current_waypoint': 0,
    'mission_running': False,
    'mission_id': None,
    'vehicle': None,
    'detection_log': [],
    'stop_requested': False
}

state_lock = threading.Lock()

# ===================== NETWORK FUNCTIONS =====================
def register_with_gcs():
    """Register drone with GCS server."""
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
        return False

def send_heartbeat():
    """Send periodic heartbeat to GCS."""
    try:
        with state_lock:
            status = 'running' if MISSION_STATE['mission_running'] else 'idle'
            if MISSION_STATE['mission_running'] and MISSION_STATE['vehicle']:
                try:
                    if MISSION_STATE['vehicle'].mode.name != 'GUIDED':
                        status = 'paused'
                except:
                    pass
        
        response = requests.post(
            f"{GCS_SERVER_URL}/drone-heartbeat",
            json={
                'drone_id': DRONE_ID,
                'current_waypoint': MISSION_STATE['current_waypoint'],
                'mission_status': status
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

def validate_waypoints_file(mission_data):
    """Validate waypoints JSON file integrity."""
    try:
        if 'mission_id' not in mission_data:
            return False, "Missing 'mission_id' field"
        if 'waypoints' not in mission_data:
            return False, "Missing 'waypoints' field"
        if 'checksum' not in mission_data:
            return False, "Missing 'checksum' field"
        
        waypoints = mission_data['waypoints']
        if not isinstance(waypoints, list):
            return False, "Waypoints must be a list"
        
        if len(waypoints) == 0:
            return False, "Waypoints list is empty"
        
        for idx, wp in enumerate(waypoints):
            if not isinstance(wp, list):
                return False, f"Waypoint {idx+1} must be a list [lat, lon, height]"
            
            if len(wp) < 2:
                return False, f"Waypoint {idx+1} must have at least latitude and longitude"
            
            lat = wp[0]
            if not isinstance(lat, (int, float)) or lat < -90 or lat > 90:
                return False, f"Waypoint {idx+1}: invalid latitude"
            
            lon = wp[1]
            if not isinstance(lon, (int, float)) or lon < -180 or lon > 180:
                return False, f"Waypoint {idx+1}: invalid longitude"
        
        # Verify checksum
        checksum_data = mission_data.copy()
        checksum_data['checksum'] = None
        json_str = json.dumps(checksum_data, sort_keys=True)
        calculated_checksum = hashlib.md5(json_str.encode('utf-8')).hexdigest()
        
        if calculated_checksum != mission_data['checksum']:
            return False, "Checksum verification failed"
        
        return True, None
    
    except Exception as e:
        return False, f"Validation error: {str(e)}"

def get_mission_from_gcs(run_dir=None):
    """Get mission waypoints from GCS by downloading JSON file."""
    try:
        response = requests.get(f"{GCS_SERVER_URL}/get-mission", timeout=5)
        if response.status_code != 200:
            return False
        
        mission_info = response.json()
        waypoints_url = mission_info.get('waypoints_url') or mission_info.get('waypoints_file')
        
        if not waypoints_url:
            print("✗ No waypoints file URL in mission info")
            return False
        
        if waypoints_url.startswith('/'):
            waypoints_url = f"{GCS_SERVER_URL}{waypoints_url}"
        else:
            waypoints_url = f"{GCS_SERVER_URL}/{waypoints_url}"
        
        print(f"Downloading waypoints file from {waypoints_url}...")
        file_response = requests.get(waypoints_url, timeout=10)
        
        if file_response.status_code != 200:
            print(f"✗ Failed to download waypoints file: {file_response.status_code}")
            return False
        
        try:
            mission_data = file_response.json()
        except json.JSONDecodeError as e:
            print(f"✗ Invalid JSON in waypoints file: {e}")
            return False
        
        is_valid, error_msg = validate_waypoints_file(mission_data)
        if not is_valid:
            print(f"✗ Waypoints file validation failed: {error_msg}")
            return False
        
        print(f"✓ Waypoints file validated successfully")
        
        mission_id = mission_data['mission_id']
        if run_dir:
            if mission_id and not run_dir.endswith(mission_id):
                run_dir = f"run_{mission_id}"
                os.makedirs(run_dir, exist_ok=True)
        
        if run_dir:
            waypoints_file_path = os.path.join(run_dir, 'waypoints.json')
            with open(waypoints_file_path, 'w') as f:
                json.dump(mission_data, f, indent=2)
            print(f"✓ Waypoints file saved to {waypoints_file_path}")
        
        waypoints = mission_data['waypoints']
        
        with state_lock:
            MISSION_STATE['waypoints'] = waypoints
            MISSION_STATE['mission_id'] = mission_id
            if mission_info.get('resume', False) and not MISSION_STATE['mission_running']:
                MISSION_STATE['current_waypoint'] = mission_info.get('current_waypoint', 0)
                print(f"Resuming mission from waypoint {MISSION_STATE['current_waypoint']}")
        
        return True
    
    except Exception as e:
        print(f"✗ Error getting mission: {e}")
        return False

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

def send_detection_log():
    """Send detection log to GCS after mission completion."""
    try:
        with state_lock:
            detection_log = MISSION_STATE['detection_log']
            mission_id = MISSION_STATE['mission_id']
        
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
            print(f"✓ Detection log sent to GCS: {response.json()}")
            return True
        else:
            print(f"✗ Failed to send log: {response.status_code}")
            return False
    except Exception as e:
        print(f"✗ Error sending detection log: {e}")
        return False

# ===================== DRONE CONTROL =====================
def get_distance_metres(aLocation1, aLocation2):
    """Calculate distance between two GPS coordinates in meters."""
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    a = math.sin(dlat*math.pi/360)**2 + math.cos(aLocation1.lat*math.pi/180) * math.cos(aLocation2.lat*math.pi/180) * math.sin(dlong*math.pi/360)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return 6371000 * c

def arm_and_takeoff(vehicle, target_altitude):
    """Arm vehicle and takeoff to target altitude."""
    print("[Mission] Pre-arm checks...")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialize...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)

    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f" Altitude: {alt:.2f}m")
        if alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def goto_gps_point_with_stop_check(vehicle, target_location, target_speed, tolerance=1.0):
    """Navigate to waypoint with periodic stop command checks."""
    print(f"Flying to: {target_location.lat}, {target_location.lon}")
    vehicle.simple_goto(target_location, groundspeed=target_speed)

    check_count = 0
    while True:
        if check_count % 2 == 0:
            if check_stop_command():
                print("⚠ STOP COMMAND RECEIVED during flight! Initiating RTL...")
                vehicle.mode = VehicleMode("RTL")
                with state_lock:
                    MISSION_STATE['stop_requested'] = True
                    MISSION_STATE['mission_running'] = False
                return True
        
        current_location = vehicle.location.global_relative_frame
        distance = get_distance_metres(current_location, target_location)
        print(f" Distance to target: {distance:.2f}m")
        if distance <= tolerance:
            print(" Target reached.")
            break
        time.sleep(1)
        check_count += 1
    
    return False

# ===================== DETECTION =====================
def classify_pothole_severity(bbox_area, confidence):
    """
    Classify pothole severity based on bounding box area and confidence.
    Returns: 'minor', 'major', or 'good'
    """
    if bbox_area < MINOR_POTHOLE_AREA_THRESHOLD:
        return 'good'
    elif bbox_area < MAJOR_POTHOLE_AREA_THRESHOLD:
        return 'minor'
    else:
        return 'major'

class WebcamVideoStream:
    def __init__(self, src=0, width=640, height=480, fps=30):
        self.stream = cv2.VideoCapture(src)
        self.stream.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.stream.set(cv2.CAP_PROP_FPS, fps)
        self.stream.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False
        self.thread = threading.Thread(target=self.update, daemon=True)

    def start(self):
        self.thread.start()
        return self

    def update(self):
        while not self.stopped:
            self.stream.grab()
            (self.grabbed, self.frame) = self.stream.retrieve()

    def read(self):
        return self.frame

    def stop(self):
        self.stopped = True
        self.thread.join()
        self.stream.release()

def run_mission_with_detection(vehicle, waypoints, altitude, speed, model):
    """Run mission with continuous pothole detection throughout entire mission."""
    video_writer = None
    vs = None
    detection_log = []
    json_path = None
    base_dir = None
    
    try:
        with state_lock:
            mission_id = MISSION_STATE.get('mission_id')
        
        if mission_id:
            base_dir = f"run_{mission_id}"
        else:
            run_timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            base_dir = f"run_{run_timestamp}"
        
        os.makedirs(base_dir, exist_ok=True)
        detected_frames_dir = os.path.join(base_dir, "detected_frames")
        os.makedirs(detected_frames_dir, exist_ok=True)
        json_path = os.path.join(base_dir, "detection_log.json")
        video_path = os.path.join(base_dir, "raw_video.mp4")

        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        video_writer = cv2.VideoWriter(video_path, fourcc, 30, (FRAME_WIDTH, FRAME_HEIGHT))

        print("Starting camera...")
        vs = WebcamVideoStream(src=CAMERA_INDEX, width=FRAME_WIDTH, height=FRAME_HEIGHT, fps=30).start()
        time.sleep(2.0)
        print("Camera initialized")

        frame_id = 0
        detection_running = True
        
        def detection_loop():
            """Continuous detection loop running in separate thread."""
            nonlocal frame_id, detection_log
            
            while detection_running:
                frame = vs.read()
                if frame is None:
                    continue
                
                frame_id += 1
                frame_draw = frame.copy()
                
                # Run YOLO inference
                if YOLO_AVAILABLE and model is not None:
                    results = model(frame, conf=SCORE_THRESHOLD, verbose=False)
                    
                    detections_found = False
                    for result in results:
                        boxes = result.boxes
                        for box in boxes:
                            # Get class ID and confidence
                            cls_id = int(box.cls[0])
                            conf = float(box.conf[0])
                            
                            # Get class name
                            cls_name = result.names[cls_id] if hasattr(result, 'names') else f"class_{cls_id}"
                            
                            # Filter for pothole-related detections
                            # Note: You may need to train a custom YOLO model for pothole detection
                            # For now, we'll detect any object and classify based on size
                            # Common COCO classes that might be relevant: 'pothole' (if custom trained)
                            
                            # Get bounding box coordinates
                            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                            
                            # Calculate area
                            bbox_area = (x2 - x1) * (y2 - y1)
                            
                            # Classify severity
                            severity = classify_pothole_severity(bbox_area, conf)
                            
                            # Only log if it's a pothole (minor or major)
                            if severity in ['minor', 'major']:
                                detections_found = True
                                
                                # Choose color based on severity
                                color = (0, 255, 255) if severity == 'minor' else (0, 0, 255)  # Yellow or Red
                                
                                # Draw bounding box
                                cv2.rectangle(frame_draw, (x1, y1), (x2, y2), color, 3)
                                
                                # Draw label
                                label_text = f"{severity.upper()} Pothole {conf:.2f}"
                                label_size = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
                                cv2.rectangle(frame_draw, (x1, y1 - label_size[1] - 10), 
                                            (x1 + label_size[0], y1), color, -1)
                                cv2.putText(frame_draw, label_text, (x1, y1 - 5), 
                                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                                
                                # Log detection
                                current_time = datetime.datetime.now().isoformat()
                                try:
                                    loc = vehicle.location.global_relative_frame
                                    log_entry = {
                                        "timestamp": current_time,
                                        "frame_id": frame_id,
                                        "object": "pothole",
                                        "severity": severity,
                                        "confidence": conf,
                                        "bbox_area": int(bbox_area),
                                        "latitude": loc.lat,
                                        "longitude": loc.lon,
                                        "altitude_m": loc.alt
                                    }
                                    detection_log.append(log_entry)
                                    print(f"{severity.upper()} pothole detected at {loc.lat:.6f}, {loc.lon:.6f} (conf: {conf:.2f})")
                                    
                                    frame_filename = os.path.join(detected_frames_dir, f"frame_{frame_id:04d}_{severity}.jpg")
                                    cv2.imwrite(frame_filename, frame_draw)
                                except Exception as e:
                                    print(f"Error getting GPS location: {e}")
                                    log_entry = {
                                        "timestamp": current_time,
                                        "frame_id": frame_id,
                                        "object": "pothole",
                                        "severity": severity,
                                        "confidence": conf,
                                        "bbox_area": int(bbox_area),
                                        "latitude": 0,
                                        "longitude": 0,
                                        "altitude_m": 0
                                    }
                                    detection_log.append(log_entry)
                
                # Write frame to video
                video_writer.write(frame_draw)
                time.sleep(0.03)  # ~30 FPS
        
        # Start detection thread
        detection_thread = threading.Thread(target=detection_loop, daemon=True)
        detection_thread.start()
        print("Detection loop started - running continuously throughout mission")
        
        # Start mission
        arm_and_takeoff(vehicle, altitude)
        print("--- Starting GPS Waypoint Mission ---")
        
        for i in range(MISSION_STATE['current_waypoint'], len(waypoints)):
            if check_stop_command():
                print("⚠ STOP COMMAND RECEIVED! Initiating RTL...")
                with state_lock:
                    MISSION_STATE['stop_requested'] = True
                    MISSION_STATE['mission_running'] = False
                detection_running = False
                vehicle.mode = VehicleMode("RTL")
                break
            
            with state_lock:
                MISSION_STATE['current_waypoint'] = i
            
            target_location = waypoints[i]
            
            print(f"\n>>> Flying to Waypoint {i+1}/{len(waypoints)}...")
            print(f"Target: lat={target_location.lat:.6f}, lon={target_location.lon:.6f}, alt={target_location.alt:.1f}m")
            
            goto_gps_point_with_stop_check(vehicle, target_location, speed)
            
            if check_stop_command():
                print("⚠ STOP COMMAND RECEIVED! Initiating RTL...")
                with state_lock:
                    MISSION_STATE['stop_requested'] = True
                    MISSION_STATE['mission_running'] = False
                detection_running = False
                vehicle.mode = VehicleMode("RTL")
                break

        detection_running = False
        print("Stopping detection loop...")
        time.sleep(1)
        
        with state_lock:
            was_stopped = MISSION_STATE['stop_requested']
        
        if was_stopped:
            print("--- Mission Stopped by Command ---")
        else:
            print("--- Mission Complete ---")
        
        print("Returning to Launch...")
        vehicle.mode = VehicleMode("RTL")
        
        print("Waiting for landing...")
        while vehicle.mode.name == "RTL":
            time.sleep(1)
        
        # Save detection log locally
        with open(json_path, 'w') as f:
            json.dump(detection_log, f, indent=4)
        
        with state_lock:
            MISSION_STATE['detection_log'] = detection_log
        
        video_writer.release()
        vs.stop()
        
        print("Sending detection log to GCS...")
        send_detection_log()
        
        with state_lock:
            MISSION_STATE['mission_running'] = False
            MISSION_STATE['current_waypoint'] = 0
        
    except KeyboardInterrupt:
        print("\n⚠ Keyboard Interrupt in mission thread! Saving data and initiating RTL...")
        detection_running = False
        time.sleep(1)
        
        if detection_log and json_path:
            try:
                with open(json_path, 'w') as f:
                    json.dump(detection_log, f, indent=4)
                print(f"✓ Detection log saved to {json_path}")
            except Exception as e:
                print(f"⚠ Error saving detection log: {e}")
        
        if video_writer:
            try:
                video_writer.release()
            except:
                pass
        
        if vs:
            try:
                vs.stop()
            except:
                pass
        
        try:
            vehicle.mode = VehicleMode("RTL")
        except:
            pass
        
        with state_lock:
            MISSION_STATE['mission_running'] = False
            MISSION_STATE['current_waypoint'] = 0
    
    except Exception as e:
        print(f"Mission error: {e}")
        import traceback
        traceback.print_exc()
        detection_running = False
        time.sleep(1)
        
        if detection_log and json_path:
            try:
                with open(json_path, 'w') as f:
                    json.dump(detection_log, f, indent=4)
            except:
                pass
        
        if video_writer:
            try:
                video_writer.release()
            except:
                pass
        if vs:
            try:
                vs.stop()
            except:
                pass
        
        try:
            vehicle.mode = VehicleMode("RTL")
        except:
            pass
        
        with state_lock:
            MISSION_STATE['mission_running'] = False

# ===================== MAIN =====================
def main():
    global GCS_SERVER_URL
    
    parser = argparse.ArgumentParser(description="Drone client for Jetson Nano with YOLO")
    parser.add_argument("--gcs-url", default=GCS_SERVER_URL, help="GCS server URL")
    parser.add_argument("--model", default=MODEL_PATH, help="Path to YOLO model file")
    parser.add_argument("--camera", type=int, default=CAMERA_INDEX, help="Camera index")
    args = parser.parse_args()
    
    GCS_SERVER_URL = args.gcs_url
    
    print(f"Drone Client Starting...")
    print(f"GCS Server: {GCS_SERVER_URL}")
    print(f"Drone ID: {DRONE_ID}")
    
    # Load YOLO model
    model = None
    if YOLO_AVAILABLE:
        try:
            print(f"Loading YOLO model from {args.model}...")
            model = YOLO(args.model)
            print("✓ YOLO model loaded successfully")
        except Exception as e:
            print(f"✗ Error loading YOLO model: {e}")
            print("Continuing without detection model...")
    else:
        print("⚠ YOLO not available. Install with: pip install ultralytics")
    
    # Register with GCS
    while not register_with_gcs():
        print("Retrying registration in 5 seconds...")
        time.sleep(5)
    
    # Start heartbeat thread
    def heartbeat_loop():
        print("Heartbeat thread started")
        while True:
            try:
                success = send_heartbeat()
                if success:
                    print(f"Heartbeat sent at {datetime.datetime.now().strftime('%H:%M:%S')}")
            except Exception as e:
                print(f"Heartbeat loop error: {e}")
            time.sleep(HEARTBEAT_INTERVAL)
    
    heartbeat_thread = threading.Thread(target=heartbeat_loop, daemon=True)
    heartbeat_thread.start()
    
    # Connect to drone
    print(f"Connecting to Pixhawk on {DRONEKIT_CONNECTION_STRING}...")
    vehicle = connect(DRONEKIT_CONNECTION_STRING, baud=DRONEKIT_BAUD, wait_ready=False, timeout=60)
    print("Drone connected!")
    
    with state_lock:
        MISSION_STATE['vehicle'] = vehicle
    
    # Main loop
    print("Waiting for mission from GCS...")
    mission_loaded = False
    
    while True:
        try:
            if not mission_loaded:
                if get_mission_from_gcs():
                    with state_lock:
                        mission_id = MISSION_STATE.get('mission_id')
                    
                    if mission_id:
                        run_dir = f"run_{mission_id}"
                        os.makedirs(run_dir, exist_ok=True)
                    
                    mission_loaded = True
                    with state_lock:
                        waypoint_count = len(MISSION_STATE['waypoints'])
                    print(f"✓ Mission loaded: {waypoint_count} waypoints")
            
            if mission_loaded and not MISSION_STATE['mission_running']:
                if check_start_command():
                    print("Start command received! Starting mission...")
                    with state_lock:
                        MISSION_STATE['mission_running'] = True
                    
                    waypoints = [LocationGlobalRelative(wp[0], wp[1], wp[2] if len(wp) > 2 else TARGET_ALTITUDE) 
                                for wp in MISSION_STATE['waypoints']]
                    
                    mission_thread = threading.Thread(
                        target=run_mission_with_detection,
                        args=(vehicle, waypoints, TARGET_ALTITUDE, TARGET_SPEED, model),
                        daemon=True
                    )
                    mission_thread.start()
            
            if mission_loaded and not MISSION_STATE['mission_running'] and MISSION_STATE['current_waypoint'] == 0:
                mission_loaded = False
                print("Mission completed. Waiting for new mission...")
            
            time.sleep(1)
            
        except KeyboardInterrupt:
            print("\n⚠ Keyboard Interrupt received! Initiating emergency shutdown...")
            break
        except Exception as e:
            print(f"Error in main loop: {e}")
            time.sleep(5)
    
    # Cleanup
    print("\n" + "=" * 60)
    print("Shutting down drone client...")
    print("=" * 60)
    
    with state_lock:
        if MISSION_STATE['mission_running']:
            print("⚠ Mission is running. Stopping mission and initiating RTL...")
            MISSION_STATE['mission_running'] = False
            MISSION_STATE['stop_requested'] = True
    
    if vehicle:
        try:
            print("Setting vehicle to RTL mode...")
            vehicle.mode = VehicleMode("RTL")
            print("✓ RTL mode activated")
        except Exception as e:
            print(f"⚠ Error setting RTL mode: {e}")
    
    print("Waiting for mission thread to cleanup...")
    time.sleep(2)
    
    if vehicle:
        try:
            vehicle.close()
            print("✓ Vehicle connection closed")
        except Exception as e:
            print(f"⚠ Error closing vehicle: {e}")
    
    print("=" * 60)
    print("Drone client stopped.")
    print("=" * 60)

if __name__ == "__main__":
    main()

