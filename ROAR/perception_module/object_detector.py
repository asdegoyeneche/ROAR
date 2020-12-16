from ROAR.utilities_module.camera_models import Camera
from ROAR.utilities_module.utilities import dist_to_line_2d
from abc import abstractmethod
from collections import deque
import logging
from typing import Any
from ROAR.agent_module.agent import Agent
from ROAR.perception_module.detector import Detector
from ROAR.perception_module.lane_detector import grayscale, canny, gaussian_blur, region_of_interest

import cv2
import math
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
from PIL import Image
import os

class ObjectDetector(Detector):
    def __init__(self, agent: Agent, camera: Camera, name: str, **kwargs):
        super().__init__(agent, **kwargs)
        self.camera = camera
        self.img_list = []
        self.name = name

    def run_in_series(self, **kwargs) -> Any:
        depth_img = self.camera.data
        self.img_list.append(depth_img)
        self.process_image(depth_img, visualize=True)

    def run_in_threaded(self, **kwargs):
        pass

    def process_image(self, image, visualize=False, **kwargs):
        if image is None:
            return None

        gray = grayscale(image)
        ret, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)

        kernel = np.ones((3,3), np.uint8)
        dilated = cv2.dilate(thresh, kernel, iterations = 1)

        processed_img = dilated


        if visualize:
            cv2.imshow(self.name + " depth img", processed_img)
            cv2.waitKey(1)

