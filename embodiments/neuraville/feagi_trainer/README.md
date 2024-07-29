# Using the FEAGI Trainer on Windows, Linux or Mac

## Requirements:

1. Python version 3.8+
2. Pip version 21+ (?)
3. Magic link (get from /brain-visualizer on NRS under main nav Embodiment -> Magic Link)
4. Command Prompt (CMD) or PowerShell (Windows) / Terminal (Mac or Linux)
5. Git (https://git-scm.com/downloads for Windows. Mac and Linux should have it by default)

## Installation

In CMD/Terminal, run:

1. `git clone https://github.com/feagi/controllers.git`
2. `cd controllers/embodiments/neuraville/feagi_trainer`
3. `python3 -m venv venv` (for Windows, `python -m venv venv`)
4. `source venv/bin/activate`
5. Once the venv is activated, run:
   - Linux / Mac:
     1. `pip3 install -r requirements.txt`
     2. `python3 controller.py --magic_link "<insert link here>" (replace <insert link here> without removing the quotes in the command).`
   - Windows:
     1. `pip install -r requirements.txt`
     2. `python controller.py --magic_link "<insert link here>" (replace <insert link here> without removing the quotes in the command).`

At this point, the websocket should be successfully connected. You should see a log of the websocket URL (`wss:// . . .`), with no errors following.

## Customizing the Trainer

### Displaying Images

- A trainer has been built to send FEAGI images and display the image in the browser with FEAGI's guess of what that image is.
- To use it, after doing the installation steps above, open a separate terminal and run the Flask server with `python utils/flask_server.py`
- You can manually test FEAGI's ID recognition output:
  1. In the brain visualizer GUI, click the + beside Cortical Areas, click Output, and select ID recognition from the dropdown.
  2. Shift + click anywhere in the ID recognition cortical area and hit space to activate. You should see "FEAGI Guess" update above the image in the separate browser window. It will change depending on the selected voxel.
