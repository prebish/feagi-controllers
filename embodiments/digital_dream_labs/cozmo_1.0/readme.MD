# How to connect FEAGI with Cozmo
Assuming you already have playground up running.
1) `git clone https://github.com/feagi/controllers.git`
2) `cd controllers/embodiments/digital_dream_labs/cozmo_1.0`
3) `python3 -m venv venv`
4) `source venv/bin/activate`
5) Connect your wifi with Cozmo.
6) Run python using below:
   1. mac/linux users:
      1. `pip3 install -r requirements`
      2. `python3 controller.py`
   2. windows users:
      1. `pip install -r requirements`
      2. `python controller.py`

# How do I turn Cozmo on and connect it?

1. Place Cozmo on the charger (repeat this step if Cozmo is already on the charger while turned off).
2. Wait until you see the green light on the back.
3. Gently lift Cozmo's arm up and then gently put it down again.
4. The Wi-Fi password will display on Cozmo's screen.
5. Use this password to connect to Wi-Fi.
6. You can now run the controller as described in step #6 above.

# Extra flags
