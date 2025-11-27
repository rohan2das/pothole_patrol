"""
GCS Server for Road Surface Anomaly Detection System
Handles KML upload, waypoint processing, mission management, and anomaly visualization
"""
from flask import Flask, render_template, request, jsonify, send_from_directory
from flask_cors import CORS
import os
import json
import hashlib
import uuid
from datetime import datetime
from pathlib import Path
import xml.etree.ElementTree as ET
from werkzeug.utils import secure_filename
import math
from collections import defaultdict

app = Flask(__name__)
CORS(app)

# Configuration
UPLOAD_FOLDER = 'uploads'
MISSIONS_FOLDER = 'missions'
ALLOWED_EXTENSIONS = {'kml', 'kmz'}
MAX_FILE_SIZE = 10 * 1024 * 1024  # 10MB

os.makedirs(UPLOAD_FOLDER, exist_ok=True)
os.makedirs(MISSIONS_FOLDER, exist_ok=True)

# Mission state
missions = {}  # mission_id -> mission_data
registered_drones = {}  # drone_id -> drone_info
anomaly_data = {}  # mission_id -> list of anomalies

def allowed_file(filename):
    return '.' in filename and filename.rsplit('.', 1)[1].lower() in ALLOWED_EXTENSIONS

def parse_kml(file_path):
    """
    Parse KML file and extract waypoints (coordinates).
    Returns list of [lat, lon, alt] waypoints.
    """
    try:
        tree = ET.parse(file_path)
        root = tree.getroot()
        
        # Handle KML namespace
        ns = {'kml': 'http://www.opengis.net/kml/2.2'}
        
        waypoints = []
        
        # Find all coordinates in the KML file
        # KML can have coordinates in various places: Placemark, LineString, etc.
        for placemark in root.findall('.//kml:Placemark', ns):
            # Try to find coordinates in Point
            point = placemark.find('.//kml:Point', ns)
            if point is not None:
                coord_elem = point.find('.//kml:coordinates', ns)
                if coord_elem is not None and coord_elem.text:
                    coords = coord_elem.text.strip().split(',')
                    if len(coords) >= 2:
                        lon = float(coords[0])
                        lat = float(coords[1])
                        alt = float(coords[2]) if len(coords) > 2 else 15.0
                        waypoints.append([lat, lon, alt])
            
            # Try to find coordinates in LineString (for paths)
            linestring = placemark.find('.//kml:LineString', ns)
            if linestring is not None:
                coord_elem = linestring.find('.//kml:coordinates', ns)
                if coord_elem is not None and coord_elem.text:
                    coord_strings = coord_elem.text.strip().split()
                    for coord_str in coord_strings:
                        coords = coord_str.split(',')
                        if len(coords) >= 2:
                            lon = float(coords[0])
                            lat = float(coords[1])
                            alt = float(coords[2]) if len(coords) > 2 else 15.0
                            waypoints.append([lat, lon, alt])
        
        # Also check for coordinates directly (some KML files don't use namespaces properly)
        if not waypoints:
            for coord_elem in root.findall('.//{http://www.opengis.net/kml/2.2}coordinates'):
                if coord_elem.text:
                    coord_strings = coord_elem.text.strip().split()
                    for coord_str in coord_strings:
                        coords = coord_str.split(',')
                        if len(coords) >= 2:
                            lon = float(coords[0])
                            lat = float(coords[1])
                            alt = float(coords[2]) if len(coords) > 2 else 15.0
                            waypoints.append([lat, lon, alt])
        
        # Fallback: try without namespace
        if not waypoints:
            for coord_elem in root.findall('.//coordinates'):
                if coord_elem.text:
                    coord_strings = coord_elem.text.strip().split()
                    for coord_str in coord_strings:
                        coords = coord_str.split(',')
                        if len(coords) >= 2:
                            lon = float(coords[0])
                            lat = float(coords[1])
                            alt = float(coords[2]) if len(coords) > 2 else 15.0
                            waypoints.append([lat, lon, alt])
        
        return waypoints
    
    except Exception as e:
        print(f"Error parsing KML: {e}")
        return []

def create_mission_file(mission_id, waypoints):
    """Create mission JSON file with checksum."""
    mission_data = {
        'mission_id': mission_id,
        'waypoints': waypoints,
        'created_at': datetime.now().isoformat()
    }
    
    # Calculate checksum
    checksum_data = mission_data.copy()
    checksum_data['checksum'] = None
    json_str = json.dumps(checksum_data, sort_keys=True)
    checksum = hashlib.md5(json_str.encode('utf-8')).hexdigest()
    mission_data['checksum'] = checksum
    
    # Save to file
    mission_file = os.path.join(MISSIONS_FOLDER, f'{mission_id}.json')
    with open(mission_file, 'w') as f:
        json.dump(mission_data, f, indent=2)
    
    return mission_data

# ===================== API ENDPOINTS =====================

@app.route('/')
def index():
    """Serve main web interface."""
    return render_template('index.html')

# Store uploaded files temporarily
uploaded_files = {}  # file_id -> filepath

@app.route('/api/upload-kml', methods=['POST'])
def upload_kml():
    """Handle KML file upload (just save, don't process yet)."""
    if 'file' not in request.files:
        return jsonify({'error': 'No file provided'}), 400
    
    file = request.files['file']
    if file.filename == '':
        return jsonify({'error': 'No file selected'}), 400
    
    if not allowed_file(file.filename):
        return jsonify({'error': 'Invalid file type. Only KML/KMZ files allowed.'}), 400
    
    try:
        # Save uploaded file
        filename = secure_filename(file.filename)
        file_id = str(uuid.uuid4())
        filepath = os.path.join(UPLOAD_FOLDER, f"{file_id}_{filename}")
        file.save(filepath)
        
        # Store file reference
        uploaded_files[file_id] = {
            'filepath': filepath,
            'filename': filename,
            'uploaded_at': datetime.now().isoformat()
        }
        
        return jsonify({
            'success': True,
            'file_id': file_id,
            'filename': filename,
            'message': 'File uploaded successfully. Click Process to extract waypoints.'
        })
    
    except Exception as e:
        return jsonify({'error': f'Error uploading file: {str(e)}'}), 500

@app.route('/api/process-kml', methods=['POST'])
def process_kml():
    """Process uploaded KML file and extract waypoints."""
    data = request.json
    file_id = data.get('file_id')
    
    if not file_id:
        return jsonify({'error': 'file_id required'}), 400
    
    if file_id not in uploaded_files:
        return jsonify({'error': 'File not found. Please upload again.'}), 404
    
    try:
        file_info = uploaded_files[file_id]
        filepath = file_info['filepath']
        
        # Parse KML to extract waypoints
        waypoints = parse_kml(filepath)
        
        if not waypoints:
            return jsonify({'error': 'No waypoints found in KML file'}), 400
        
        # Create mission
        mission_id = str(uuid.uuid4())
        mission_data = create_mission_file(mission_id, waypoints)
        
        # Store mission
        missions[mission_id] = {
            'mission_id': mission_id,
            'waypoints': waypoints,
            'status': 'pending',
            'created_at': datetime.now().isoformat(),
            'kml_filename': file_info['filename']
        }
        anomaly_data[mission_id] = []
        
        # Clean up uploaded file reference
        del uploaded_files[file_id]
        
        return jsonify({
            'success': True,
            'mission_id': mission_id,
            'waypoint_count': len(waypoints),
            'waypoints': waypoints[:10]  # Return first 10 for preview
        })
    
    except Exception as e:
        return jsonify({'error': f'Error processing KML: {str(e)}'}), 500

@app.route('/api/missions')
def list_missions():
    """List all missions."""
    return jsonify({
        'missions': [
            {
                'mission_id': m['mission_id'],
                'waypoint_count': len(m['waypoints']),
                'status': m['status'],
                'created_at': m['created_at'],
                'anomaly_count': len(anomaly_data.get(m['mission_id'], []))
            }
            for m in missions.values()
        ]
    })

@app.route('/api/mission/<mission_id>')
def get_mission(mission_id):
    """Get mission details."""
    if mission_id not in missions:
        return jsonify({'error': 'Mission not found'}), 404
    
    mission = missions[mission_id]
    return jsonify({
        'mission_id': mission_id,
        'waypoints': mission['waypoints'],
        'status': mission['status'],
        'anomalies': anomaly_data.get(mission_id, [])
    })

@app.route('/api/mission/<mission_id>/start', methods=['POST'])
def start_mission(mission_id):
    """Start a mission."""
    if mission_id not in missions:
        return jsonify({'error': 'Mission not found'}), 404
    
    missions[mission_id]['status'] = 'running'
    return jsonify({'success': True, 'status': 'running'})

@app.route('/api/mission/<mission_id>/stop', methods=['POST'])
def stop_mission(mission_id):
    """Stop a mission."""
    if mission_id not in missions:
        return jsonify({'error': 'Mission not found'}), 404
    
    missions[mission_id]['status'] = 'stopped'
    return jsonify({'success': True, 'status': 'stopped'})

@app.route('/get-mission')
def get_mission_for_drone():
    """Get mission waypoints for drone client (legacy endpoint)."""
    # Find the most recent running or pending mission
    active_mission = None
    for mission in missions.values():
        if mission['status'] in ['running', 'pending']:
            active_mission = mission
            break
    
    if not active_mission:
        return jsonify({'error': 'No active mission'}), 404
    
    mission_id = active_mission['mission_id']
    mission_file = os.path.join(MISSIONS_FOLDER, f'{mission_id}.json')
    
    if os.path.exists(mission_file):
        with open(mission_file, 'r') as f:
            mission_data = json.load(f)
        
        return jsonify({
            **mission_data,
            'waypoints_url': f'/missions/{mission_id}.json',
            'stop_requested': missions[mission_id]['status'] == 'stopped'
        })
    
    return jsonify({'error': 'Mission file not found'}), 404

@app.route('/missions/<filename>')
def serve_mission_file(filename):
    """Serve mission JSON file."""
    return send_from_directory(MISSIONS_FOLDER, filename)

@app.route('/drone-register', methods=['POST'])
def register_drone():
    """Register a drone with the GCS."""
    data = request.json
    drone_id = data.get('drone_id')
    
    if not drone_id:
        return jsonify({'error': 'drone_id required'}), 400
    
    registered_drones[drone_id] = {
        'drone_id': drone_id,
        'registered_at': datetime.now().isoformat(),
        'last_heartbeat': datetime.now().isoformat(),
        'status': 'idle'
    }
    
    return jsonify({'success': True, 'drone_id': drone_id})

@app.route('/drone-heartbeat', methods=['POST'])
def drone_heartbeat():
    """Receive heartbeat from drone."""
    data = request.json
    drone_id = data.get('drone_id')
    
    if drone_id in registered_drones:
        registered_drones[drone_id]['last_heartbeat'] = datetime.now().isoformat()
        registered_drones[drone_id]['status'] = data.get('mission_status', 'idle')
        registered_drones[drone_id]['current_waypoint'] = data.get('current_waypoint', 0)
    
    return jsonify({'success': True})

@app.route('/drone-status')
def drone_status():
    """Get drone status (for checking start command)."""
    # Find active mission
    active_mission = None
    for mission in missions.values():
        if mission['status'] == 'running':
            active_mission = mission
            break
    
    mission_running = active_mission is not None
    
    return jsonify({
        'mission_running': mission_running,
        'mission_id': active_mission['mission_id'] if active_mission else None
    })

@app.route('/mission-complete', methods=['POST'])
def mission_complete():
    """Receive detection log from drone after mission completion."""
    data = request.json
    drone_id = data.get('drone_id')
    mission_id = data.get('mission_id')
    detection_log = data.get('detection_log', [])
    
    if mission_id not in missions:
        return jsonify({'error': 'Mission not found'}), 404
    
    # Store anomalies
    anomaly_data[mission_id] = detection_log
    missions[mission_id]['status'] = 'completed'
    
    print(f"Received {len(detection_log)} detections for mission {mission_id}")
    
    return jsonify({
        'success': True,
        'anomalies_received': len(detection_log)
    })

@app.route('/api/anomalies/<mission_id>')
def get_anomalies(mission_id):
    """Get anomalies for a specific mission."""
    if mission_id not in anomaly_data:
        return jsonify({'anomalies': []})
    
    return jsonify({'anomalies': anomaly_data[mission_id]})

def get_distance_metres(lat1, lon1, lat2, lon2):
    """Calculate distance between two GPS coordinates in meters."""
    dlat = lat2 - lat1
    dlong = lon2 - lon1
    a = math.sin(dlat*math.pi/360)**2 + math.cos(lat1*math.pi/180) * math.cos(lat2*math.pi/180) * math.sin(dlong*math.pi/360)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return 6371000 * c

def aggregate_road_conditions():
    """
    Aggregate all anomalies from all missions and create road segments with condition ratings.
    Returns road segments with color coding based on pothole density and severity.
    """
    all_anomalies = []
    
    # Collect all anomalies from all missions
    for mission_id, anomalies in anomaly_data.items():
        for anomaly in anomalies:
            if anomaly.get('latitude') and anomaly.get('longitude'):
                all_anomalies.append({
                    'lat': anomaly['latitude'],
                    'lon': anomaly['longitude'],
                    'severity': anomaly.get('severity', 'minor'),
                    'confidence': anomaly.get('confidence', 0.5),
                    'mission_id': mission_id
                })
    
    if not all_anomalies:
        return []
    
    # Group anomalies by proximity (create road segments)
    # Segment size: ~100 meters
    SEGMENT_SIZE = 100  # meters
    segments = defaultdict(lambda: {
        'anomalies': [],
        'total_count': 0,
        'major_count': 0,
        'minor_count': 0,
        'avg_lat': 0,
        'avg_lon': 0
    })
    
    # Simple clustering: group nearby anomalies
    for anomaly in all_anomalies:
        # Find nearest segment or create new one
        segment_key = None
        min_dist = float('inf')
        
        for key, segment in segments.items():
            dist = get_distance_metres(
                segment['avg_lat'], segment['avg_lon'],
                anomaly['lat'], anomaly['lon']
            )
            if dist < SEGMENT_SIZE and dist < min_dist:
                min_dist = dist
                segment_key = key
        
        if segment_key is None:
            # Create new segment
            segment_key = f"{anomaly['lat']:.6f},{anomaly['lon']:.6f}"
            segments[segment_key]['avg_lat'] = anomaly['lat']
            segments[segment_key]['avg_lon'] = anomaly['lon']
        
        # Add to segment
        segment = segments[segment_key]
        segment['anomalies'].append(anomaly)
        segment['total_count'] += 1
        if anomaly['severity'] == 'major':
            segment['major_count'] += 1
        else:
            segment['minor_count'] += 1
        
        # Update average position
        n = segment['total_count']
        segment['avg_lat'] = (segment['avg_lat'] * (n-1) + anomaly['lat']) / n
        segment['avg_lon'] = (segment['avg_lon'] * (n-1) + anomaly['lon']) / n
    
    # Convert to road condition segments with color coding
    road_segments = []
    for key, segment in segments.items():
        # Determine condition based on density and severity
        total = segment['total_count']
        major = segment['major_count']
        
        # Color coding logic:
        # Green: no potholes or very few minor ones
        # Yellow: some minor potholes or 1-2 major ones
        # Red: many potholes or multiple major ones
        if total == 0:
            condition = 'good'
            color = '#28a745'  # Green
        elif major >= 2 or total >= 5:
            condition = 'poor'
            color = '#dc3545'  # Red
        elif major >= 1 or total >= 3:
            condition = 'moderate'
            color = '#ffc107'  # Yellow
        else:
            condition = 'good'
            color = '#28a745'  # Green
        
        road_segments.append({
            'lat': segment['avg_lat'],
            'lon': segment['avg_lon'],
            'condition': condition,
            'color': color,
            'total_potholes': total,
            'major_potholes': major,
            'minor_potholes': segment['minor_count']
        })
    
    return road_segments

@app.route('/road-map')
def road_map():
    """Serve road conditions map view."""
    return render_template('road_map.html')

@app.route('/api/road-conditions')
def get_road_conditions():
    """Get aggregated road conditions for all missions."""
    segments = aggregate_road_conditions()
    
    # Also get all individual anomalies for detailed view
    all_anomalies = []
    for mission_id, anomalies in anomaly_data.items():
        for anomaly in anomalies:
            if anomaly.get('latitude') and anomaly.get('longitude'):
                all_anomalies.append(anomaly)
    
    return jsonify({
        'segments': segments,
        'anomalies': all_anomalies,
        'total_segments': len(segments),
        'total_anomalies': len(all_anomalies)
    })

if __name__ == '__main__':
    print("Starting GCS Server...")
    print("Access the web UI at: http://localhost:8000")
    app.run(host='0.0.0.0', port=8000, debug=True)

