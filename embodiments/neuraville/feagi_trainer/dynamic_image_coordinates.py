import time
from models import LatestStatic

def get_static_data(static):
    if not static: 
        return LatestStatic(
            image_id="",
            feagi_image_id="",
            correct_count=0,
            incorrect_count=0,
            no_reply_count=0,
            image_dimensions="",
            raw_image_dimensions="",
            last_image_time=None,
            last_feagi_time=None,
            loop=None,
            image_display_duration=None,
            image_path=None,
            test_mode=None,
            image_gap_duration=None
        )
    return static

def update_image_ids(new_image_id=None, new_feagi_image_id=None, static=None):
    static = get_static_data(static)

    # Get existing values or set defaults
    image_id = static.image_id
    feagi_image_id = static.feagi_image_id
    correct_count = static.correct_count
    incorrect_count = static.incorrect_count
    no_reply_count = static.no_reply_count
    image_dimensions = static.image_dimensions
    raw_image_dimensions = static.raw_image_dimensions
    last_image_time = static.last_image_time
    last_feagi_time = static.last_feagi_time

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
    return LatestStatic(
        image_id=image_id,
        feagi_image_id=feagi_image_id,
        correct_count=correct_count,
        incorrect_count=incorrect_count,
        no_reply_count=no_reply_count,
        image_dimensions=image_dimensions,
        raw_image_dimensions=raw_image_dimensions,
        last_image_time=last_image_time,
        last_feagi_time=last_feagi_time,
        loop=static.loop,
        image_display_duration=static.image_display_duration,
        image_path=static.image_path,
        test_mode=static.test_mode,
        image_gap_duration=static.image_gap_duration
    )
