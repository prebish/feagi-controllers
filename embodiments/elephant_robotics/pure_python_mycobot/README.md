# Connect with mycobot
### IMPORTANT: Assuming you have already updated based on the MyCobot requirement. Also make sure that you are already running FEAGI and the Godot bridge.
## Local or playground
1) git clone https://github.com/feagi/feagi-connector.git
2) cd feagi-connector/embodiments/elephant_robotics/pure_python_mycobot
3) python3 -m venv venv
4) source venv/bin/activate
 5) Find the type of usb mycobot uses. 

mac/linux users:
pip3 install -r requirements
python3 controller.py

windows users:
pip install -r requirements
python controller.py --magic <insert link here>
