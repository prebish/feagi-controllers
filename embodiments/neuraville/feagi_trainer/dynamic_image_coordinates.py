import time
from models import empty_latest_static, LatestStatic


def update_image_ids(new_image_id=None, new_feagi_image_id=None, static=None):
    static = static if static else empty_latest_static

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

    # Conditionally update stats
    if new_image_id is not None:
        image_id = new_image_id
        last_image_time = time.time()

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
        feagi_controlled=static.feagi_controlled,
        loop=static.loop,
        image_display_duration=static.image_display_duration,
        image_path=static.image_path,
        test_mode=static.test_mode,
        image_gap_duration=static.image_gap_duration,
    )
