import cv2 # OpenCV

def process_image(raw_frame):
    # Define the coordinates for the box
    top_left = (160, 60)
    bottom_right = (400, 240)

    # Define the border thickness
    border_thickness = 3

    # Draw the outer black rectangle (border)
    cv2.rectangle(raw_frame, 
                (top_left[0] - border_thickness, top_left[1] - border_thickness),
                (bottom_right[0] + border_thickness, bottom_right[1] + border_thickness),
                (0, 0, 0), border_thickness)

    # Draw the inner green rectangle
    cv2.rectangle(raw_frame, top_left, bottom_right, (0, 255, 0), 2)

    # Add text to the image in the top left of the box
    text = "FEAGI Perception: dog"
    text_position = (top_left[0], top_left[1] + 5)  # Adjusted to fit background

    # Font and text size
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.6
    font_thickness = 2

    # Calculate the width and height of the text box
    (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, font_thickness)
    text_height += baseline

    # Create a rectangle filled with black color for the text background
    background_top_left = (text_position[0], text_position[1] - text_height)
    background_bottom_right = (text_position[0] + text_width, text_position[1] + baseline)
    cv2.rectangle(raw_frame, background_top_left, background_bottom_right, (0, 0, 0), cv2.FILLED)

    # Put the text over the black rectangle
    cv2.putText(raw_frame, text, text_position, font, font_scale, (0, 255, 0), font_thickness)

    # Save the processed image to a file
    image_path = 'latest_image.jpg'
    cv2.imwrite(image_path, raw_frame)
    print("Image updated")
