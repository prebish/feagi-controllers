import json

image_id = ''
feagi_image_id = ''

def update_image_ids(new_image_id=None, new_feagi_image_id=None):
    global image_id, feagi_image_id
    
    if new_image_id is not None:
        image_id = new_image_id
    if new_feagi_image_id is not None:
        feagi_image_id = new_feagi_image_id

    data = {
        'image_id': image_id,
        'feagi_image_id': feagi_image_id
    }
    
    with open('image_ids.json', 'w') as f:
        json.dump(data, f)

def get_latest_ids():
    try:
        with open('image_ids.json', 'r') as f:
            data = json.load(f)
        return data['image_id'], data['feagi_image_id']
    except (FileNotFoundError, json.JSONDecodeError, KeyError):
        return '', ''  # Return empty strings if file doesn't exist or is invalid