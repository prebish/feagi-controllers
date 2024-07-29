# For Windows, Linux or Mac with Feagi_trainer
1) `git clone https://github.com/feagi/controllers.git`
2) `cd controllers/embodiments/neuraville/feagi_trainer`
3) `python3 -m venv venv` (python -m venv venv for windows) 
4) `source venv/bin/activate`
5) Run python using below:
   1. linux users:
      1. `pip3 install -r requirements.txt`
      2. `python3 controller.py --magic_link "<insert link here>" (replace <insert link here> without removing the quotes in the command).`
   2. windows users:
      1. `pip install -r requirements.txt`
      2. `python controller.py --magic_link "<insert link here>" (replace <insert link here> without removing the quotes in the command).`

# Quick steps for experienced and advanced users
1) python3 controller.py

# Requirements:
1) python 3.8+
2) Magic link (you can find this from NRS)
3) CMD (WINDOWS) or Terminal (MAC OR LINUX)
4) Git (https://git-scm.com/downloads for windows. Mac and Linux should have it by default)

# I don't see any change in behavior.
Do you have the image inside the folder? If not, you will need to grab an image and place it in the folder where the controller is located, or follow the path specified in your configuration.json's `image_path`.

# How do I add more pictures?
Your image's name should be formatted like this: `#-#-#`. For example, `0-0-1`, `1-0-0`, `0-2-3`. They all need to be in an image format such as PNG, JPEG, JPG, or any other static format. You can add them without restarting your trainer, as it is designed to work with dynamic file changes.

# I want the image to stay on for a certain period. How do I do that?
Go to `image_display_duration` in your configuration.json. Update it to the number of seconds you want the image to stay on display.

# I want to pause for a certain period. How do I do that?
Go to `image_gap_duration` in your configuration.json and update it to the number of seconds you want the image to stay off.

# Extra flags
Example command: `python controller.py --help`
```commandline
optional arguments:
  -h, --help            Show this help message and exit.
  
  -magic_link MAGIC_LINK, --magic_link MAGIC_LINK
                        Use a magic link. You can find your magic link from NRS studio.
                        
  -magic-link MAGIC_LINK, --magic-link MAGIC_LINK
                        Use a magic link. You can find your magic link from NRS studio.
                        
  -magic MAGIC, --magic MAGIC
                        Use a magic link. You can find your magic link from NRS studio.
                        
  -ip IP, --ip IP       Specify the FEAGI IP address.
  
  -port PORT, --port PORT
                        Change the ZMQ port. Use 30000 for Docker and 3000 for localhost.

```