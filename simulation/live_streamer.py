# live_streamer.py
import threading
import cv2
import json
from flask import Flask, Response, jsonify, render_template_string
import queue

app = Flask(__name__)

import logging
log = logging.getLogger('werkzeug')
log.disabled = True
log.setLevel(logging.ERROR)

app.logger.disabled = True

frame_queue = queue.Queue(maxsize=10)
latest_frame = None
latest_occupancy = {}

# --- NEW: Built-in HTML Dashboard ---
HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>UParking Live Dashboard</title>
    <style>
        body { font-family: Arial, sans-serif; background-color: #121212; color: #ffffff; text-align: center; }
        .container { display: flex; flex-direction: row; justify-content: center; gap: 20px; padding: 20px; }
        .video-feed { border: 3px solid #333; border-radius: 8px; width: 640px; height: 480px; background: #000; }
        .data-panel { background: #1e1e1e; padding: 20px; border-radius: 8px; width: 300px; text-align: left; overflow-y: auto; max-height: 480px; }
        .spot { padding: 8px; margin: 5px 0; border-radius: 4px; font-weight: bold; }
        .occupied { background-color: #8b0000; color: #ffcccc; }
        .empty { background-color: #006400; color: #ccffcc; }
    </style>
</head>
<body>
    <h2>UParking Simulation Live Feed</h2>
    <div class="container">
        <div>
            <h3>Drone Vision</h3>
            <img class="video-feed" src="/video_feed" alt="Waiting for drone camera..." />
        </div>
        <div class="data-panel">
            <h3>Live Occupancy</h3>
            <div id="occupancy-list">Waiting for scan data...</div>
        </div>
    </div>

    <script>
        // Fetch occupancy data every 2 seconds and update the list
        setInterval(() => {
            fetch('/occupancy_data')
                .then(response => response.json())
                .then(data => {
                    const list = document.getElementById('occupancy-list');
                    let entries = Object.entries(data);
                    
                    if(entries.length === 0) { 
                        list.innerHTML = 'Scanning in progress...';
                        return;
                    }
                    list.innerHTML = '';                   
                    entries.sort((a, b) => {
                        return a[0] , localeCompare(b[0], undefined, {
                            numeric: true,
                            sesnsitivity: 'base'
                        });
                    });
                    
                    entries.forEach(([label, occupied]) => {
                        let div = document.createElement('div');
                        div.className = 'spot ' + (occupied ? 'occupied' : 'empty');
                        div.innerText = `${label}: ${occupied ? 'OCCUPIED' : 'EMPTY'}`;
                        list.appendChild(div);
                    });
                });
                .catch(err => console.log(."waiting for server..."));
        }, 2000);
    </script>
</body>
</html>
"""

def update_state(scan_log):
    global latest_occupancy
    for entry in scan_log:
        latest_occupancy[entry["label"]] = entry["occupied"]

def generate_frames():
    global latest_frame
    while True:
        try:
            frame = frame_queue.get(timeout=1.0)
            latest_frame = frame
        except queue.Empty:
            frame = latest_frame

        if frame is None:
            continue

        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret: continue
            
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.route('/')
def index():
    """Serves the main dashboard homepage."""
    return render_template_string(HTML_TEMPLATE)

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/occupancy_data')
def occupancy_data():
    return jsonify(latest_occupancy)

def start_server():
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)

def launch_streamer_thread():
    t = threading.Thread(target=start_server, daemon=True)
    t.start()
    return frame_queue, update_state