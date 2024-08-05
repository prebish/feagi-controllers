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
    loop: Optional[bool] = None
    image_display_duration: Optional[float] = None
    image_path: Optional[str] = None
    test_mode: Optional[bool] = None
    image_gap_duration: Optional[float] = None