# Runs a server that displays the latest processed image in the browser with HTML
from flask import Flask, send_file, render_template_string
import os

app = Flask(__name__)

@app.route('/')
def index():
    html = '''
    <!DOCTYPE html>
    <html>
    <head>
        <title>FEAGI Image Viewer</title>
        <style>
            body { display: flex; justify-content: center; align-items: center; height: 100vh; margin: 0; background-color: black; }
            img { max-height:90vh; min-height: 500px; border:1px solid white; }
        </style>
    </head>
    <body>
            <img id="feagi-image" src="/image" alt="FEAGI Image" id="image">
        <script>
            setInterval(function() {
                document.getElementById('feagi-image').src = '/image?' + new Date().getTime();
            }, 1000);  // Refresh every 1 second
        </script>
    </body>
    </html>
    '''
    return render_template_string(html)

@app.route('/image')
def get_image():
    image_path = os.path.join(os.path.dirname(__file__), 'latest_image.jpg')
    if os.path.exists(image_path):
        return send_file(image_path, mimetype='image/jpeg')
    else:
        return "Image not found", 404

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)