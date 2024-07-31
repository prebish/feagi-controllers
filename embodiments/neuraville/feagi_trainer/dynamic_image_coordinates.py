import time

# Use latest static data or start from base values
def get_static_data(static_data):
    if not static_data: 
        return {
            "image_id": "",
            "feagi_image_id": "",
            "correct_count": 0,
            "incorrect_count": 0,
            "no_reply_count": 0,
            "last_image_time": None,
            "last_feagi_time": None
        }
    return static_data


# Update IDs and counts
def update_image_ids(new_image_id=None, new_feagi_image_id=None, static={}):
    static = get_static_data(static)

    if static:
        # Get existing values or set defaults
        image_id = static.get('image_id', '')
        feagi_image_id = static.get('feagi_image_id', '')
        correct_count = static.get('correct_count', 0)
        incorrect_count = static.get('incorrect_count', 0)
        no_reply_count = static.get('no_reply_count', 0)
        last_image_time = static.get('last_image_time', None)
        last_feagi_time = static.get('last_feagi_time', None)

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

        # Return latest static
        return {
            'image_id': image_id,
            'feagi_image_id': feagi_image_id,
            'correct_count': correct_count,
            'incorrect_count': incorrect_count,
            'no_reply_count': no_reply_count,
            'last_image_time': last_image_time,
            'last_feagi_time': last_feagi_time
        }
    else:
        print('No data found to update.')
        return static
