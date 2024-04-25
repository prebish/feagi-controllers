# Connecting with Petoi

Petoi has 9 degrees of freedom servos. It relies on Pyserial, which uses USB, and also utilizes Bluetooth for wireless connectivity. This setup can work with the new brand of Petoi as well. The IMU feature is found under the advanced settings. Simply connecting your Petoi to the playground will allow you to interact with the servos only. To use the IMU, you will need to navigate to the advanced section.

# Connect with Bluetooth
This assumes you are already familiar with the playground and how to use it. Here is how you can connect with Petoi:

1) Click on "Embodiment."
2) Select "Bluetooth Robot."
3) Choose your Petoi.
4) You're connected! Enjoy!

You can perform the same steps on the NRS website as well.

# Connect with USB
This is a bit different from Bluetooth and is more advanced, suited for those with a programming background. If the steps are unclear or if you have any questions, please create an issue. When creating the issue, add the title "Pyserial Petoi" followed by your own title.

Here is how you connect with Petoi:

1) Plug your computer into Petoi using a USB cable.
2) Make sure that you are in `feagi-agent/embodiments/petoi/pyserial`.
3) Run the command: `python3 controller.py`.
4) That's all!

# Any issues? Create an issue!
To ensure that everyone's (especially your) experience goes smoothly, please don't hesitate to 
create an issue. 
