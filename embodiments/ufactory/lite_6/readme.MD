# For Windows, Linux, or Mac with Mycobot and FEAGI
1. `git clone https://github.com/feagi/feagi-connector.git`
2. `cd feagi/embodiments/ufactory/lite_6`
3. `python3 -m venv venv`
4. `source venv/bin/activate`
5. Connect your ufactory arm.
6. Run Python using the commands below:
   1. Mac/Linux users:
      1. `pip3 install -r requirements`
   2. Windows users:
      1. `pip install -r requirements`
7. `git clone git@github.com:xArm-Developer/xArm-Python-SDK.git xArm`
8. `pip3 install -r requirements.txt`
9. `cd xArm && python setup.py install`
10. `cd ..`
11. For Mac/Linux: `python3 controller.py --magic_link "< replace here with square bracket>"`
    For Windows: `python controller.py --magic_link "< replace here with square bracket>"`