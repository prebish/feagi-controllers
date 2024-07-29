import json
import time
import os

file_path = 'utils/image_training_data.json'

static_data = {
        "image_id": "",
        "feagi_image_id": "",
        "correct_count": 0,
        "incorrect_count": 0,
        "no_reply_count": 0,
        "last_image_time": None,
        "last_feagi_time": None
    }

# Create JSON file if it doesn't exist
def create_file_if_not_exists(file_path):
    if not os.path.exists(file_path):
        with open(file_path, 'w') as file:
            json.dump(static_data, file, indent=4)

# Read data from JSON file
def read_from_json(file_path):
    if os.path.exists(file_path):
        with open(file_path, 'r') as file:
            try:
                return json.load(file)
            except json.JSONDecodeError:
                return None
    else:
        return None

# Update a single key in the JSON file
def update_single_key(file_path, key, value):
    # Read existing data
    if os.path.exists(file_path):
        with open(file_path, 'r') as file:
            try:
                data = json.load(file)
            except json.JSONDecodeError:
                data = {}
    else:
        data = {}
    # Update the specific key with the new value
    data[key] = value
    # Write the updated data back to the file
    with open(file_path, 'w') as file:
        json.dump(data, file, indent=4)

# Write new data to JSON file
def write_to_json(file_path, new_data):
    if os.path.exists(file_path):
        with open(file_path, 'r') as file:
            try:
                data = json.load(file)
            except json.JSONDecodeError:
                data = {}
    else:
        data = {}

    data.update(new_data)

    with open(file_path, 'w') as file:
        json.dump(data, file, indent=4)

# Get latest IDs
def get_latest_ids():
    create_file_if_not_exists(file_path)
    data = read_from_json(file_path)
    
    if data is not None:
        # Extract only the required fields
        return {
            'image_id': data.get('image_id', ''),
            'feagi_image_id': data.get('feagi_image_id', ''),
            'correct_count': data.get('correct_count', 0),
            'incorrect_count': data.get('incorrect_count', 0),
            'no_reply_count': data.get('no_reply_count', 0)
        }
    else:
        # Return default values if data is None
        return {
            'image_id': '',
            'feagi_image_id': '',
            'correct_count': 0,
            'incorrect_count': 0,
            'no_reply_count': 0
        }

# Update IDs and counts
def update_image_ids(new_image_id=None, new_feagi_image_id=None):
    create_file_if_not_exists(file_path)
    data = read_from_json(file_path)
    
    if data is not None:
        # Get existing values or set defaults
        image_id = data.get('image_id', '')
        feagi_image_id = data.get('feagi_image_id', '')
        correct_count = data.get('correct_count', 0)
        incorrect_count = data.get('incorrect_count', 0)
        no_reply_count = data.get('no_reply_count', 0)
        last_image_time = data.get('last_image_time', None)
        last_feagi_time = data.get('last_feagi_time', None)
        
        # Conditionally update image_id and last_image_time
        if new_image_id is not None:
            image_id = new_image_id
            last_image_time = time.time()
        
        # Conditionally update feagi_image_id, last_feagi_time, and correct_count or incorrect_count
        if new_feagi_image_id is not None:
            feagi_image_id = new_feagi_image_id
            last_feagi_time = time.time()
            if new_feagi_image_id == image_id:
                correct_count += 1
            else:
                incorrect_count += 1
        
        # Increment no_reply_count if the last image time is newer than the last FEAGI time
        if last_image_time is not None:
            if last_feagi_time is None or last_image_time > last_feagi_time:
                no_reply_count += 1
        
        # Update data with new values
        updated_data = {
            'image_id': image_id,
            'feagi_image_id': feagi_image_id,
            'correct_count': correct_count,
            'incorrect_count': incorrect_count,
            'no_reply_count': no_reply_count,
            'last_image_time': last_image_time,
            'last_feagi_time': last_feagi_time
        }
        
        # Write updated data back to the file
        write_to_json(file_path, updated_data)

        # Return latest data
        return updated_data
    else:
        print('No data found to update.')
        return static_data
