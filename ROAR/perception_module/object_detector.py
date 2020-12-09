from ROAR.utilities_module.camera_models import Camera
from ROAR.utilities_module.utilities import dist_to_line_2d
from abc import abstractmethod
from collections import deque
import logging
from typing import Any
from ROAR.agent_module.agent import Agent
from ROAR.perception_module.detector import Detector

import cv2
import math
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
from PIL import Image
import os

class ObjectDetector(Detector):
    def __init__(self, agent: Agent, camera: Camera, **kwargs):
        super().__init__(agent, **kwargs)
        self.camera = camera

    def run_in_series(self, **kwargs) -> Any:
        depth_img = self.camera.data
        self.process_image(depth_img, visualize=True)

    def run_in_threaded(self, **kwargs):
        pass

    def process_image(self, image, visualize=False, **kwargs):
        if image is None or len(image.shape) != 3:
            return None

        processed_img = np.copy(image)

        if visualize:
            cv2.imshow("processed img", processed_img)
            cv2.waitKey(1)

