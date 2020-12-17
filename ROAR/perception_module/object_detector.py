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
    def __init__(self, agent: Agent, camera: Camera, name: str, **kwargs):
        super().__init__(agent, **kwargs)
        self.camera = camera
        self.prev_img = None
        self.name = name

    def run_in_series(self, **kwargs) -> Any:
        depth_img = self.camera.data
        self.process_image(depth_img, visualize=True)

    def run_in_threaded(self, **kwargs):
        pass

    def process_image(self, image, visualize=False, **kwargs):
        if image is None:
            return None

        gray_img = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

        if self.prev_img is None:
            self.prev_img = gray_img
            return None

        diff_image = cv2.absdiff(gray_img, self.prev_img)
        ret, thresh = cv2.threshold(diff_image, 60, 255, cv2.THRESH_BINARY)

        kernel = np.ones((3, 3), np.uint8)
        dilated = cv2.dilate(thresh, kernel, iterations=1)

        contours, heirarchy = cv2.findContours(dilated.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        valid_contours = []
        for cntr in contours:
            x, y, w, h = cv2.boundingRect(cntr)
            if (y >= 250) and (cv2.contourArea(cntr) >= 30):
                valid_contours.append(cntr)

        if visualize:
            processed_img = cv2.drawContours(image.copy(), valid_contours, -1, (0, 255, 0), 3)
            cv2.imshow(self.name, processed_img)
            cv2.waitKey(1)

        self.prev_img = gray_img
