from pydantic import BaseModel
from typing import Optional


class LatestStatic(BaseModel):
    image_id: str = ""
    feagi_image_id: str = ""
    correct_count: int = 0
    incorrect_count: int = 0
    no_reply_count: int = 0
    image_dimensions: str = ""
    raw_image_dimensions: str = ""
    last_image_time: Optional[float] = None
    last_feagi_time: Optional[float] = None
    feagi_controlled: Optional[bool] = None
    loop: Optional[bool] = None
    image_display_duration: Optional[float] = None
    image_path: Optional[str] = None
    test_mode: Optional[bool] = None
    image_gap_duration: Optional[float] = None


empty_latest_static = LatestStatic(
    image_id="",
    feagi_image_id="",
    correct_count=0,
    incorrect_count=0,
    no_reply_count=0,
    image_dimensions="",
    raw_image_dimensions="",
    last_image_time=None,
    last_feagi_time=None,
    feagi_controlled=None,
    image_display_duration=None,
    image_gap_duration=None,
    loop=None,
    image_path=None,
    test_mode=None,
)
