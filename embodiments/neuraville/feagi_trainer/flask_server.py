# Creates a browser window to show user images sent to FEAGI, its selections of objects within them, etc.
import cv2
import time
import logging
import numpy as np
from flask import Flask, Response, render_template_string, jsonify

# logging.basicConfig(level=logging.DEBUG)
log = logging.getLogger('werkzeug')
log.setLevel(logging.CRITICAL)

app = Flask(__name__)

start_time = time.time()

latest_static = {
        "image_id": "",
        "feagi_image_id": "",
        "correct_count": 0,
        "incorrect_count": 0,
        "no_reply_count": 0,
        "image_dimensions": "",
        "raw_image_dimensions": "",
        "last_image_time": None,
        "last_feagi_time": None,
    }
latest_image = []
latest_raw_image = []

@app.route('/')
def index():
    runtime = time.time() - start_time
    html = '''
        <!DOCTYPE html>
        <html>
        <head>
            <title>FEAGI Image Viewer</title>
            <style>
                body {
                    max-width: 100vw;
                    overflow-x: hidden;
                    display: flex;
                    flex-direction: column;
                    justify-content: center;
                    align-items: center;
                    min-height: 100vh;
                    margin: 0;
                    background-color: black;
                    color: white;
                }
                h2, h3 {
                    color: lightgrey;
                    font-weight: 500;
                }
                h3, h4 {
                    margin-top: 0px;
                    margin-bottom: 0px;
                }
                span {
                    color: white;
                }
                img {
                    max-height: 450px;
                    max-width: 450px;
                    min-height: 32px;
                    min-width: 32px;
                    border: 1px solid white;
                }
                button {
                    align-self: center;
                    display: inline-block;
                    min-height: 28px;
                    padding: 4px 8px;
                    outline: 0;
                    border: 0;
                    border-radius: 4px;
                    font-size: 14px;
                    font-weight: 500;
                    color: white;
                    background-color: rgb(25, 118, 210);
                    cursor: pointer;
                }
                button:hover {
                    background-color: rgb(33, 150, 243);
                }
                #raw-image-parent {
                    max-width: 400px;
                    max-height: 400px;
                }
                .header {
                    width: 100%;
                    margin-bottom: 10px;
                    display: flex;
                    justify-content: space-between;
                    align-items: center;
                }
                .image-container {
                    min-width: 175px;
                    min-height: 175px;
                    padding: 10px;
                    display: flex; 
                    flex-direction: column; 
                    align-items: center;
                    border: 1px solid #404040;
                    border-radius: 5px;
                }
                .stats-container {
                    min-width: 175px;
                    max-height: 250px;
                    padding: 10px;
                    display: flex;
                    flex-direction: column;
                    gap: 5px;
                    background-color: #262626;
                    border: 1px solid #404040;
                    border-radius: 5px;
                }
            </style>
        </head>
        <body>
            <div style="display: flex; gap: 20px;">
                <div class="stats-container">
                    <button id="reset-button" title="Reset all visible data and timer">Reset Counters</button>
                    <h3 style="margin-top: 5px">Runtime: <span id="runtime">--:--:--</span></h3>
                    <h2 style="margin: 0; margin-top: 10px" title="Correct vs no reply + incorrect counts">Fitness: <span id="fitness-percent" style="font-size: 1.2rem">0%</span></h2>
                    <h3>Correct: <span id="correct-count">{{ correct_count }}</span></h3>
                    <h3>Incorrect: <span id="incorrect-count">{{ incorrect_count }}</span></h3>
                    <h3>No Reply: <span id="no-reply-count">{{ no_reply_count }}</span></h3>
                </div>

                <div class="image-container">
                    <h3 style="align-self: margin-bottom: 5px">Correct Image: <span id="image-id">{{ image_id }}</span></h3>
                    <h3>FEAGI Guess: <span id="feagi-image-id">{{ feagi_image_id }}</span></h3>
                    <div style="display: flex; align-items: flex-end; gap: 10px;">
                        <div class="image-hider">
                            <h3 style="text-align: center; color: white;">Image FEAGI Sees <span id="image-dimensions">{{ image_width }} x {{ image_height }}</span></h3>
                            <div id="image-parent" style="width: 100%; display: flex; justify-content: center;">  
                                <img src="{{ url_for('video_feed') }}" alt="video feed"/> 
                            </div>
                        </div>
                        <div class="image-hider">
                            <h3 style="text-align: center; color: white;">Actual Image <span id="raw-image-dimensions">{{ raw_image_width }} x {{ raw_image_height }}</span></h3>
                            <div id="raw-image-parent" style="width: 100%; display: flex; justify-content: center;">  
                                <img src="{{ url_for('raw_frame_feed') }}" alt="raw frame"/> 
                            </div>
                        </div>
                        <div id="unsupported-message" style="display: none; color: red;">
                            Image/video display is not supported in Firefox.
                         </div>
                    </div>
                </div>
            </div>
            <script>
                // Don't show video feed if Firefox
                document.addEventListener('DOMContentLoaded', function() {
                    let userAgent = navigator.userAgent;
                    if (userAgent.match(/firefox|fxios/i)) {
                        Array.from(document.getElementsByClassName('image-hider')).forEach(function(el) {
                            el.style.display = 'none';
                        });
                        document.getElementById('unsupported-message').style.display = 'block';
                    } 
                });

                // Set startTime to last stored value or the current time
                let startTime = localStorage.getItem('startTime') ? parseInt(localStorage.getItem('startTime')) : new Date().getTime();

                function updateContent() {
                    fetch('/latest_ids')
                        .then(response => response.json())
                        .then(data => {
                            document.getElementById('image-id').innerText = data.image_id || "N/A";
                            document.getElementById('feagi-image-id').innerText = data.feagi_image_id || "N/A";
                            document.getElementById('correct-count').innerText = data.correct_count !== undefined ? data.correct_count : '?';
                            document.getElementById('incorrect-count').innerText = data.incorrect_count !== undefined ? data.incorrect_count : '?';
                            document.getElementById('no-reply-count').innerText = data.no_reply_count !== undefined ? data.no_reply_count : '?';
                            document.getElementById('image-dimensions').innerText = data.image_dimensions || "N/A";
                            document.getElementById('raw-image-dimensions').innerText = data.raw_image_dimensions || "N/A";
                            const total = data.correct_count + data.incorrect_count + data.no_reply_count;
                            const percentCorrect = total === 0 ? 0 : data.correct_count ? (data.correct_count / total) * 100 : "?";
                            document.getElementById('fitness-percent').innerText = isFinite(percentCorrect) ? `${percentCorrect.toFixed(2)}%` : "N/A";
                        });
                }

                updateContent();  // Initial load
                setInterval(updateContent, 10);  // Refetch frequency

                function formatTime(seconds) {
                    const hours = Math.floor(seconds / 3600);
                    const minutes = Math.floor((seconds % 3600) / 60);
                    const secs = Math.floor(seconds % 60);
                    return `${String(hours).padStart(2, '0')}:${String(minutes).padStart(2, '0')}:${String(secs).padStart(2, '0')}`;
                }

                function updateRuntime() {
                    const runtimeInSeconds = (new Date().getTime() - startTime) / 1000;
                    document.getElementById('runtime').innerText = formatTime(runtimeInSeconds);
                }

                setInterval(updateRuntime, 1000); 
                updateRuntime();

                function resetTimerAndData() {
                    fetch('/reset_timer_and_data')
                        .then(response => response.json())
                        .then(data => {
                            startTime = new Date().getTime(); 
                            localStorage.setItem('startTime', startTime); 
                        });
                }

                document.getElementById('reset-button').addEventListener('click', resetTimerAndData);

                // Clear localStorage when the server starts (this can be added to a separate page load check or server start logic)
                window.addEventListener('load', () => {
                    resetTimerAndData();
                });
            </script>
        </body>
        </html>
    '''
    return render_template_string(html, 
                              runtime=f"{runtime:.2f}",
                              image_id=latest_static["image_id"],
                              feagi_image_id=latest_static["feagi_image_id"],
                              correct_count=latest_static["correct_count"],
                              incorrect_count=latest_static["incorrect_count"],
                              no_reply_count=latest_static["no_reply_count"],
                              image_dimensions=latest_static["image_dimensions"],
                              raw_image_dimensions=latest_static["raw_image_dimensions"])

# Process latest image for HTML display
def gen(use_raw=True):
    while True:
        if use_raw:
            data = latest_raw_image
        else:
            data = latest_image
        if isinstance(data, np.ndarray):
            ret, buffer = cv2.imencode('.jpg', data)
            if ret:
                frame = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

# Fetch latest image sent to FEAGI
@app.route('/video_feed')
def video_feed():
    return Response(gen(False),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

# Fetch latest raw image
@app.route('/raw_frame_feed')
def raw_frame_feed():
    return Response(gen(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


# Fetch latest image ID and any FEAGI recognition ID
@app.route('/latest_ids')
def latest_ids():
    global latest_static
    return jsonify(latest_static)


# Reset timer and data
@app.route('/reset_timer_and_data')
def reset_timer_and_data():
    global start_time, latest_static
    start_time = time.time()

    latest_static = {
        "image_id": "",
        "feagi_image_id": "",
        "correct_count": 0,
        "incorrect_count": 0,
        "no_reply_count": 0,
        "image_dimensions": "",
        "raw_image_dimensions": "",
        "last_image_time": None,
        "last_feagi_time": None
    }

    return jsonify({'status': 'success', 'start_time': start_time, 'reset_data': latest_static})


def start_app():
    app.run(host='0.0.0.0', port=4001, debug=False, use_reloader=False)

