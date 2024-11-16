"""

Facial expression definitions.

Based on the "Expressive Eyes" project by Catherine Chambers:
https://git.brl.ac.uk/ca2-chambers/expressive-eyes

"""

from typing import Optional, List
import time
from collections import deque
import pycozmo
import numpy as np
from PIL import Image
from pycozmo.procedural_face import ProceduralFace, DEFAULT_WIDTH, DEFAULT_HEIGHT
face_selected = deque()
eye_one_location = deque()
eye_two_location = deque()

__all__ = [
    "Neutral",
    "Anger",
    "Sadness",
    "Happiness",
    "Surprise",
    "Disgust",
    "Fear",
    "Pleading",
    "Vulnerability",
    "Despair",
    "Guilt",
    "Disappointment",
    "Embarrassment",
    "Horror",
    "Skepticism",
    "Annoyance",
    "Fury",
    "Suspicion",
    "Rejection",
    "Boredom",
    "Tiredness",
    "Asleep",
    "Confusion",
    "Amazement",
    "Excitement",
    "Excitement2"
]


async def expressions(cli):
    expressions_array = [
        Neutral(),
        Excitement2(),
        Anger(),
        Sadness(),
        Happiness(),
        Surprise(),
        Disgust(),
        Fear(),
        Pleading(),
        Vulnerability(),
        Despair(),
        Guilt(),
        Disappointment(),
        Embarrassment(),
        Horror(),
        Skepticism(),
        Annoyance(),
        Fury(),
        Suspicion(),
        Rejection(),
        Boredom(),
        Tiredness(),
        Asleep(),
        Confusion(),
        Amazement(),
        Excitement()
    ]
    face_ignor_threshold = 1
    last_face_expression_time = time.time()
    while True:
        if face_selected:
            if time.time() - last_face_expression_time > face_ignor_threshold:
                last_face_expression_time = time.time()
                face_generator = pycozmo.procedural_face.interpolate(
                    Neutral(), expressions_array[face_selected[0]],
                    pycozmo.robot.FRAME_RATE * 2)
                for face in face_generator:
                    # expressions_array[0].eyes[0].lids[1].y -= 0.1
                    # expressions_array[0].eyes[0].lids[1].bend -= 0.1
                    # expressions_array[0].eyes[0].lids[0].angle += 25.0
                    # expressions_array[0].eyes[1].upper_inner_radius_x += 1.0
                    # expressions_array[0].eyes[0].upper_inner_radius_x += 1.0
                    # expressions_array[0].eyes[0].scale_x += 1.25
                    # expressions_array[0].eyes[1].upper_outer_radius_x = 1.0
                    if eye_one_location:
                        expressions_array[0].eyes[0].center_x = eye_one_location[0][0]
                        expressions_array[0].eyes[0].center_y = eye_one_location[0][1]
                        eye_one_location.pop()
                    if eye_two_location:
                        expressions_array[0].eyes[1].center_x = eye_two_location[0][0]
                        expressions_array[0].eyes[1].center_y = eye_two_location[0][1]
                        eye_two_location.pop()
                    # Render face image.
                    im = face.render()
                    # The Cozmo protocol expects a 128x32 image, so take only the even lines.
                    np_im = np.array(im)
                    np_im2 = np_im[::2]
                    im2 = Image.fromarray(np_im2)
                    # Display face image.
                    cli.display_image(im2)
            face_selected.pop()
            if len(face_selected) > 2:
                temp = face_selected.pop()
                face_selected.clear()
                face_selected.append(temp)
        else:
            time.sleep(0.05)

class Neutral(ProceduralFace):
    def __init__(self, params: Optional[List[float]] = None, width: int = DEFAULT_WIDTH,
                 height: int = DEFAULT_HEIGHT):
        super().__init__(params, width, height)
        self.eyes[0].scale_x = 0.8
        self.eyes[0].scale_y = 0.8
        self.eyes[1].scale_x = 0.8
        self.eyes[1].scale_y = 0.8


# Six universal expressions by Ekman - https://en.wikipedia.org/wiki/Facial_expression

class Anger(ProceduralFace):

    def __init__(self,
                 params: Optional[List[float]] = None,
                 width: int = DEFAULT_WIDTH,
                 height: int = DEFAULT_HEIGHT
                 ):
        super().__init__(params, width, height)
        self.eyes[0].lids[0].y = 0.6
        self.eyes[0].lids[0].angle = -30.0
        self.eyes[1].lids[0].y = 0.6
        self.eyes[1].lids[0].angle = 30.0


class Sadness(ProceduralFace):

    def __init__(self,
                 params: Optional[List[float]] = None,
                 width: int = DEFAULT_WIDTH,
                 height: int = DEFAULT_HEIGHT
                 ):
        super().__init__(params, width, height)
        self.eyes[0].lids[0].y = 0.6
        self.eyes[0].lids[0].angle = 20.0
        self.eyes[1].lids[0].y = 0.6
        self.eyes[1].lids[0].angle = -20.0


class Happiness(ProceduralFace):

    def __init__(self,
                 params: Optional[List[float]] = None,
                 width: int = DEFAULT_WIDTH,
                 height: int = DEFAULT_HEIGHT
                 ):
        super().__init__(params, width, height)
        self.eyes[0].upper_outer_radius_x = 1.0
        self.eyes[0].upper_inner_radius_x = 1.0
        self.eyes[0].lids[1].y = 0.4
        self.eyes[0].lids[1].bend = 0.4
        self.eyes[1].upper_outer_radius_x = 1.0
        self.eyes[1].upper_inner_radius_x = 1.0
        self.eyes[1].lids[1].y = 0.4
        self.eyes[1].lids[1].bend = 0.4


class Surprise(ProceduralFace):

    def __init__(self,
                 params: Optional[List[float]] = None,
                 width: int = DEFAULT_WIDTH,
                 height: int = DEFAULT_HEIGHT
                 ):
        super().__init__(params, width, height)
        self.eyes[0].scale_x = 1.25
        self.eyes[0].scale_y = 1.25
        self.eyes[1].scale_x = 1.25
        self.eyes[1].scale_y = 1.25


class Disgust(ProceduralFace):

    def __init__(self,
                 params: Optional[List[float]] = None,
                 width: int = DEFAULT_WIDTH,
                 height: int = DEFAULT_HEIGHT
                 ):
        super().__init__(params, width, height)
        self.eyes[0].lids[0].y = 0.3
        self.eyes[0].lids[0].angle = 10.0
        self.eyes[0].lids[1].y = 0.3
        self.eyes[1].lids[0].y = 0.2
        self.eyes[1].lids[0].angle = 20.0
        self.eyes[1].lids[1].y = 0.2
        self.eyes[1].lids[1].angle = 10.0


class Fear(ProceduralFace):

    def __init__(self,
                 params: Optional[List[float]] = None,
                 width: int = DEFAULT_WIDTH,
                 height: int = DEFAULT_HEIGHT
                 ):
        super().__init__(params, width, height)
        self.eyes[0].lids[0].angle = 30.0
        self.eyes[0].lids[0].bend = 0.1
        self.eyes[0].lids[1].y = 0.4
        self.eyes[0].lids[1].angle = 10.0
        self.eyes[1].lids[0].angle = -30.0
        self.eyes[1].lids[0].bend = 0.1
        self.eyes[1].lids[1].y = 0.4
        self.eyes[1].lids[1].angle = -10.0


# Sub-expressions of sadness.

class Pleading(ProceduralFace):

    def __init__(self,
                 params: Optional[List[float]] = None,
                 width: int = DEFAULT_WIDTH,
                 height: int = DEFAULT_HEIGHT
                 ):
        super().__init__(params, width, height)
        self.eyes[0].lids[0].angle = 30.0
        self.eyes[0].lids[1].y = 0.5
        self.eyes[1].lids[0].angle = -30.0
        self.eyes[1].lids[1].y = 0.5


class Vulnerability(ProceduralFace):

    def __init__(self,
                 params: Optional[List[float]] = None,
                 width: int = DEFAULT_WIDTH,
                 height: int = DEFAULT_HEIGHT
                 ):
        super().__init__(params, width, height)
        self.eyes[0].lids[0].angle = 20.0
        self.eyes[0].lids[0].y = 0.3
        self.eyes[0].lids[1].angle = 10.0
        self.eyes[0].lids[1].y = 0.5
        self.eyes[1].lids[0].angle = -20.0
        self.eyes[1].lids[0].y = 0.3
        self.eyes[1].lids[1].angle = -10.0
        self.eyes[1].lids[1].y = 0.5


class Despair(ProceduralFace):

    def __init__(self,
                 params: Optional[List[float]] = None,
                 width: int = DEFAULT_WIDTH,
                 height: int = DEFAULT_HEIGHT
                 ):
        super().__init__(params, width, height)
        self.eyes[0].lids[0].angle = 30.0
        self.eyes[0].lids[0].y = 0.6
        self.eyes[1].lids[0].angle = -30.0
        self.eyes[1].lids[0].y = 0.6


class Guilt(ProceduralFace):

    def __init__(self,
                 params: Optional[List[float]] = None,
                 width: int = DEFAULT_WIDTH,
                 height: int = DEFAULT_HEIGHT
                 ):
        super().__init__(params, width, height)
        self.eyes[0].lids[0].angle = 10.0
        self.eyes[0].lids[0].y = 0.6
        self.eyes[0].lids[0].bend = 0.3
        self.eyes[1].lids[0].angle = -10.0
        self.eyes[1].lids[0].y = 0.6
        self.eyes[1].lids[0].bend = 0.3


class Disappointment(ProceduralFace):

    def __init__(self,
                 params: Optional[List[float]] = None,
                 width: int = DEFAULT_WIDTH,
                 height: int = DEFAULT_HEIGHT
                 ):
        super().__init__(params, width, height)
        self.eyes[0].lids[0].angle = -10.0
        self.eyes[0].lids[0].y = 0.3
        self.eyes[0].lids[1].y = 0.4
        self.eyes[1].lids[0].angle = 10.0
        self.eyes[1].lids[0].y = 0.3
        self.eyes[1].lids[1].y = 0.4


class Embarrassment(ProceduralFace):

    def __init__(self,
                 params: Optional[List[float]] = None,
                 width: int = DEFAULT_WIDTH,
                 height: int = DEFAULT_HEIGHT
                 ):
        super().__init__(params, width, height)
        self.eyes[0].lids[0].angle = 10.0
        self.eyes[0].lids[0].y = 0.5
        self.eyes[0].lids[0].bend = 0.1
        self.eyes[0].lids[1].y = 0.1
        self.eyes[1].lids[0].angle = -10.0
        self.eyes[1].lids[0].y = 0.5
        self.eyes[1].lids[0].bend = 0.1
        self.eyes[1].lids[1].y = 0.1


# Sub-expressions of disgust.

class Horror(ProceduralFace):

    def __init__(self,
                 params: Optional[List[float]] = None,
                 width: int = DEFAULT_WIDTH,
                 height: int = DEFAULT_HEIGHT
                 ):
        super().__init__(params, width, height)
        self.eyes[0].lids[0].angle = 20.0
        self.eyes[1].lids[0].angle = -20.0


# Sub-expressions of anger.

class Skepticism(ProceduralFace):

    def __init__(self,
                 params: Optional[List[float]] = None,
                 width: int = DEFAULT_WIDTH,
                 height: int = DEFAULT_HEIGHT
                 ):
        super().__init__(params, width, height)
        self.eyes[0].lids[0].angle = -10.0
        self.eyes[0].lids[0].y = 0.4
        self.eyes[1].lids[0].angle = 25.0
        self.eyes[1].lids[0].y = 0.15


class Annoyance(ProceduralFace):

    def __init__(self,
                 params: Optional[List[float]] = None,
                 width: int = DEFAULT_WIDTH,
                 height: int = DEFAULT_HEIGHT
                 ):
        super().__init__(params, width, height)
        self.eyes[0].lids[0].angle = -30.0
        self.eyes[0].lids[1].angle = -10.0
        self.eyes[0].lids[1].y = 0.3
        self.eyes[1].lids[0].angle = 30.0
        self.eyes[1].lids[0].y = 0.2
        self.eyes[1].lids[1].angle = 5.0
        self.eyes[1].lids[1].y = 0.4
        self.eyes[1].upper_inner_radius_x = 1.0
        self.eyes[1].upper_outer_radius_x = 1.0


class Fury(ProceduralFace):
    """ aka "enragement". """

    def __init__(self,
                 params: Optional[List[float]] = None,
                 width: int = DEFAULT_WIDTH,
                 height: int = DEFAULT_HEIGHT
                 ):
        super().__init__(params, width, height)
        self.eyes[0].lids[0].angle = -30.0
        self.eyes[0].lids[0].y = 0.3
        self.eyes[0].lids[1].y = 0.4
        self.eyes[1].lids[0].angle = 30.0
        self.eyes[1].lids[0].y = 0.3
        self.eyes[1].lids[1].y = 0.4


class Suspicion(ProceduralFace):

    def __init__(self,
                 params: Optional[List[float]] = None,
                 width: int = DEFAULT_WIDTH,
                 height: int = DEFAULT_HEIGHT
                 ):
        super().__init__(params, width, height)
        self.eyes[0].lids[0].angle = -10.0
        self.eyes[0].lids[0].y = 0.4
        self.eyes[0].lids[1].y = 0.5
        self.eyes[1].lids[0].angle = 10.0
        self.eyes[1].lids[0].y = 0.4
        self.eyes[1].lids[1].y = 0.5


class Rejection(ProceduralFace):

    def __init__(self,
                 params: Optional[List[float]] = None,
                 width: int = DEFAULT_WIDTH,
                 height: int = DEFAULT_HEIGHT
                 ):
        super().__init__(params, width, height)
        self.eyes[0].lids[0].angle = 25.0
        self.eyes[0].lids[0].y = 0.8
        self.eyes[1].lids[0].angle = 25.0
        self.eyes[1].lids[0].y = 0.8


# Sub expressions of negative emotions.

class Boredom(ProceduralFace):

    def __init__(self,
                 params: Optional[List[float]] = None,
                 width: int = DEFAULT_WIDTH,
                 height: int = DEFAULT_HEIGHT
                 ):
        super().__init__(params, width, height)
        self.eyes[0].lids[0].y = 0.4
        self.eyes[1].lids[0].y = 0.4


class Tiredness(ProceduralFace):

    def __init__(self,
                 params: Optional[List[float]] = None,
                 width: int = DEFAULT_WIDTH,
                 height: int = DEFAULT_HEIGHT
                 ):
        super().__init__(params, width, height)
        self.eyes[0].lids[0].angle = 5.0
        self.eyes[0].lids[0].y = 0.4
        self.eyes[0].lids[1].y = 0.5
        self.eyes[1].lids[0].angle = -5.0
        self.eyes[1].lids[0].y = 0.4
        self.eyes[1].lids[1].y = 0.5


class Asleep(ProceduralFace):

    def __init__(self,
                 params: Optional[List[float]] = None,
                 width: int = DEFAULT_WIDTH,
                 height: int = DEFAULT_HEIGHT
                 ):
        super().__init__(params, width, height)
        self.eyes[0].center_y = 50.0
        self.eyes[0].lids[0].y = 0.45
        self.eyes[0].lids[1].y = 0.5
        self.eyes[1].center_y = 50.0
        self.eyes[1].lids[0].y = 0.45
        self.eyes[1].lids[1].y = 0.5


# Sub-expressions of confusion.

class Confusion(ProceduralFace):

    def __init__(self,
                 params: Optional[List[float]] = None,
                 width: int = DEFAULT_WIDTH,
                 height: int = DEFAULT_HEIGHT
                 ):
        super().__init__(params, width, height)
        self.eyes[0].lids[1].y = 0.2
        self.eyes[0].lids[1].bend = 0.2
        self.eyes[1].lids[0].angle = -10.0
        self.eyes[1].lids[0].y = 0.3
        self.eyes[1].lids[1].angle = 5.0
        self.eyes[1].lids[1].y = 0.2
        self.eyes[1].lids[1].bend = 0.2


class Amazement(ProceduralFace):

    def __init__(self,
                 params: Optional[List[float]] = None,
                 width: int = DEFAULT_WIDTH,
                 height: int = DEFAULT_HEIGHT
                 ):
        super().__init__(params, width, height)
        self.eyes[0].lids[1].y = 0.2
        self.eyes[1].lids[1].y = 0.2


class Excitement(ProceduralFace):

    def __init__(self,
                 params: Optional[List[float]] = None,
                 width: int = DEFAULT_WIDTH,
                 height: int = DEFAULT_HEIGHT
                 ):
        super().__init__(params, width, height)
        self.eyes[0].lids[1].y = 0.3
        self.eyes[0].lids[1].bend = 0.2
        self.eyes[1].lids[1].y = 0.3
        self.eyes[1].lids[1].bend = 0.2


class Excitement2(ProceduralFace):

    def __init__(self,
                 params: Optional[List[float]] = None,
                 width: int = DEFAULT_WIDTH,
                 height: int = DEFAULT_HEIGHT
                 ):
        super().__init__(params, width, height)
        self.eyes[0].lids[1].y = 0.9
        self.eyes[0].lids[1].bend = 0.2
        self.eyes[1].lids[1].y = 0.9
        self.eyes[1].lids[1].bend = 0.2
