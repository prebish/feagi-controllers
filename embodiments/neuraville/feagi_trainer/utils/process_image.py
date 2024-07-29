# Modifies an image and saves it for sending
import cv2 

def get_corners(location_data, original_size=(32, 32), target_size=(60, 60)):
    # Initialize variables to store min and max coordinates
    min_x = min_y = float('inf')
    max_x = max_y = float('-inf')

    # Iterate through the JSON object
    for key in location_data.keys():
        x, y, _ = map(int, key.split('-'))
        if x < min_x:
            min_x = x
        if x > max_x:
            max_x = x
        if y < min_y:
            min_y = y
        if y > max_y:
            max_y = y

    # Flip the y-coordinates
    flipped_min_y = original_size[1] - max_y
    flipped_max_y = original_size[1] - min_y

    # Calculate top-left and bottom-right coordinates
    top_left = (min_x, flipped_min_y)
    bottom_right = (max_x, flipped_max_y)

    # Scale coordinates to target size
    scale_x = target_size[0] / original_size[0]
    scale_y = target_size[1] / original_size[1]

    top_left_scaled = (int(top_left[0] * scale_x), int(top_left[1] * scale_y))
    bottom_right_scaled = (int(bottom_right[0] * scale_x), int(bottom_right[1] * scale_y))

    return top_left_scaled, bottom_right_scaled


def process_image(image, location_data=None):
    copied_frame = image.copy()

    # Resize the image while maintaining aspect ratio
    target_width = 500
    original_height, original_width = copied_frame.shape[:2]
    aspect_ratio = original_width / original_height
    target_height = int(target_width / aspect_ratio)
    
    resized_frame = cv2.resize(copied_frame, (target_width, target_height))

    if location_data:
        # Get coordinates from location_data
        top_left, bottom_right = get_corners(location_data, original_size=(32, 32), target_size=resized_frame.shape[1::-1])
        # top_left = (160, 60)
        # bottom_right = (400, 240)

        # Define the border thickness
        border_thickness = 3

        # Draw the outer black rectangle (border)
        cv2.rectangle(resized_frame, 
                    (top_left[0] - border_thickness, top_left[1] - border_thickness),
                    (bottom_right[0] + border_thickness, bottom_right[1] + border_thickness),
                    (0, 0, 0), border_thickness)

        # Draw the inner green rectangle
        cv2.rectangle(resized_frame, top_left, bottom_right, (0, 255, 0), 2)

        # Add text to the image in the top left of the box
        # text = "FEAGI Perception: dog"
        # text_position = (top_left[0], top_left[1] + 5)  # Adjusted to fit background

        # # Font and text size
        # font = cv2.FONT_HERSHEY_SIMPLEX
        # font_scale = 0.6
        # font_thickness = 2

        # # Calculate the width and height of the text box
        # (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, font_thickness)
        # text_height += baseline

        # # Create a rectangle filled with black color for the text background
        # background_top_left = (text_position[0], text_position[1] - text_height)
        # background_bottom_right = (text_position[0] + text_width, text_position[1] + baseline)
        # cv2.rectangle(resized_frame, background_top_left, background_bottom_right, (0, 0, 0), cv2.FILLED)

        # # Put the text over the black rectangle
        # cv2.putText(resized_frame, text, text_position, font, font_scale, (0, 255, 0), font_thickness)

    # Save the processed image to a file
    image_path = 'utils/latest_image.jpg'
    cv2.imwrite(image_path, resized_frame)
    # print("Image updated")