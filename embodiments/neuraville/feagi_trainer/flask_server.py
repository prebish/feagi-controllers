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

latest_static = {}
latest_data = []


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
                h3 {
                    margin-top: 0px;
                    margin-bottom: 0px;
                }
                span {
                    color: white;
                }
                img {
                    max-height: 90vh;
                    min-height: 64px;
                    min-width: 64px;
                    border: 1px solid white;
                }
                button {
                    border: 0;
                    outline: 0;
                    cursor: pointer;
                    color: white;
                    background-color: rgb(25, 118, 210);
                    border-radius: 4px;
                    font-size: 14px;
                    font-weight: 500;
                    padding: 4px 8px;
                    display: inline-block;
                    min-height: 28px;
                }
                button:hover {
                    background-color: rgb(33, 150, 243);
                }
                .header {
                    width: 100%;
                    margin-bottom: 10px;
                    display: flex;
                    justify-content: space-between;
                    align-items: center;
                }
                .image-container {
                    padding: 10px;
                    display: flex; 
                    flex-direction: column; 
                    align-items: center;
                    border: 1px solid #404040;
                    border-radius: 5px;
                }
                .stats {
                    display: flex;
                    gap: 5px;
                    margin-bottom: 10px;
                }
                .stats-container {
                    padding: 10px;
                    display: flex;
                    flex-direction: column;
                    justify-content: center;
                    align-items: center;
                    background-color: #262626;
                    border: 1px solid #404040;
                    border-radius: 5px;
                }
            </style>
        </head>
        <body>
            <div style="display: flex; gap: 20px;">
                <!-- Stats -->
                <div class="stats-container">
                    <div class="header">
                        <div style="width: 55px;"></div>
                        <h2 style="margin: 0;">Fitness <span id="fitness-percent" style="font-size: 1.2rem">0%</span></h2>
                        <button id="reset-button" title="Reset all visible data and timer">Reset</button>
                    </div>
                    <div class="stats">
                        <h3>Correct: <span id="correct-count">{{ correct_count }}</span></h3>
                        <h3>Incorrect: <span id="incorrect-count">{{ incorrect_count }}</span></h3>
                        <h3>No Reply: <span id="no-reply-count">{{ no_reply_count }}</span></h3>
                    </div>
                    <h3>Runtime: <span id="runtime">--:--:--</span></h3>
                </div>
                <!-- Image -->
                <div class="image-container">
                    <h2>
                        Correct Image: <span id="image-id">{{ image_id }}</span> 
                        FEAGI Guess: <span id="feagi-image-id">{{ feagi_image_id }}</span>
                    </h2>
                    <h1>Image FEAGI Sees</h1>
                    <div style="width: 100%; display: flex; justify-content: center;">  
                        <img src="{{ url_for('video_feed') }}"/>
                    </div>
                </div>
            </div>
            <script>
                // Check if a localStorage key exists and use it, otherwise use current time
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
                            const total = data.correct_count + data.incorrect_count + data.no_reply_count;
                            const percentCorrect = total === 0 ? 0 : data.correct_count ? (data.correct_count / total) * 100 : "?";
                            document.getElementById('fitness-percent').innerText = percentCorrect ? `${percentCorrect.toFixed(2)}%` : "N/A";
                        });
                }

                setInterval(updateContent, 1000);  // Refresh every 1 second
                updateContent();  // Initial load

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

                setInterval(updateRuntime, 1000);  // Update runtime display every 1 second
                updateRuntime();  // Initial runtime display

                function resetTimerAndData() {
                    fetch('/reset_timer_and_data')
                        .then(response => response.json())
                        .then(data => {
                            startTime = new Date().getTime(); // Reset startTime
                            localStorage.setItem('startTime', startTime); // Save new startTime in localStorage
                            console.log('Timer and data reset on server:', data);
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
    return render_template_string(html, runtime=f"{runtime:.2f}")


def gen():
    global latest_data
    while True:
        if isinstance(latest_data, np.ndarray):
            ret, buffer = cv2.imencode('.jpg', latest_data)
            if ret:
                frame = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

    video_capture.release()

# Fetch latest image sent to FEAGI
@app.route('/video_feed')
def video_feed():
    """Video streaming route. Put this in the src attribute of an img tag."""
    print(latest_data)
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
        "last_image_time": None,
        "last_feagi_time": None
    }

    return jsonify({'status': 'success', 'start_time': start_time, 'reset_data': latest_static})


def start_app():
    app.run(host='0.0.0.0', port=4001, debug=False, use_reloader=False)

