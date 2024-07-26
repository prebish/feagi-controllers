from flask import Flask, send_file, render_template_string
import os
import dynamic_image_coordinates as img_coords
import logging

logging.basicConfig(level=logging.DEBUG)

app = Flask(__name__)

@app.route('/')
def index():
    image_id, feagi_image_id = img_coords.get_latest_ids()
    html = '''
    <!DOCTYPE html>
    <html>
    <head>
        <title>FEAGI Image Viewer</title>
        <style>
            body { display: flex; flex-direction: column; justify-content: center; align-items: center; height: 100vh; margin: 0; background-color: black; color: white; }
            h2 {color: lightgrey}
            span {color: white}
            img { max-height:90vh; min-height: 32px; border:1px solid white; }
        </style>
    </head>
    <body>
        <h2>Correct Image: <span id="image-id">{{image_id}}</span> FEAGI Guess: <span id="feagi-image-id">{{ feagi_image_id }}</span></h2>
        <img id="feagi-image" src="/image" alt="FEAGI Image">
        <script>
            function updateContent() {
                fetch('/latest_ids')
                    .then(response => response.json())
                    .then(data => {
                        document.getElementById('image-id').innerText = data.image_id;
                        document.getElementById('feagi-image-id').innerText = data.feagi_image_id || "N/A";
                    });
                document.getElementById('feagi-image').src = '/image?' + new Date().getTime();
            }
            setInterval(updateContent, 1000);  // Refresh every 1 second
            updateContent();  // Initial load
        </script>
    </body>
    </html>
    '''
    return render_template_string(html, image_id=image_id, feagi_image_id=feagi_image_id)

@app.route('/image')
def get_image():
    image_path = os.path.join(os.path.dirname(__file__), 'latest_image.jpg')
    if os.path.exists(image_path):
        return send_file(image_path, mimetype='image/jpeg')
    else:
        return "Image not found", 404

@app.route('/latest_ids')
def latest_ids():
    image_id, feagi_image_id = img_coords.get_latest_ids()
    return {
        'image_id': image_id,
        'feagi_image_id': feagi_image_id
    }

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
