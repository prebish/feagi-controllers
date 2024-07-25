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
            body { display: flex; justify-content: center; align-items: center; height: 100vh; margin: 0; background-color: #f0f0f0; }
            #image-container { box-shadow: 0 0 10px rgba(0,0,0,0.3); }
        </style>
    </head>
    <body style="background-color:black;">
        <div id="image-container">
            <img id="feagi-image" src="/image" alt="FEAGI Image" style="max-height:800px; border:1px solid white;">
        </div>
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
    image_path = 'latest_image.jpg'
    if os.path.exists(image_path):
        return send_file(image_path, mimetype='image/jpeg')
    else:
        return "Image not found", 404

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)