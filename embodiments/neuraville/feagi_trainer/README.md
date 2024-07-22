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
   1. Linux / Mac:
      1. `pip3 install -r requirements.txt`
      2. `python3 controller.py --magic_link "<insert link here>" (replace <insert link here> without removing the quotes in the command).`
   2. Windows:
      1. `pip install -r requirements.txt`
      2. `python controller.py --magic_link "<insert link here>" (replace <insert link here> without removing the quotes in the command).`

At this point, the websocket should be successfully connected.

## Customizing the Trainer

### Displaying an Image

-
