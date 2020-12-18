from ROAR.utilities_module.camera_models import Camera
from typing import Any
from ROAR.agent_module.agent import Agent
from ROAR.perception_module.detector import Detector

import cv2
import numpy as np


def region_of_interest(vertices, mask):
    if len(mask.shape) == 2:
        cv2.fillPoly(mask, vertices, 255)
    else:
        cv2.fillPoly(mask, vertices, (255,) * mask.shape[2])
    return mask


def get_vertices(img, left_prop, right_prop, bot_prop=1.0, top_prop=0.0):
    rows, cols = img.shape[:2]
    left_bottom = [cols * left_prop, rows * bot_prop]
    right_bottom = [cols * right_prop, rows * bot_prop]
    left_top = [cols * left_prop, rows * top_prop]
    right_top = [cols * right_prop, rows * top_prop]
    vertices = np.array(
        [[left_bottom, left_top, right_top, right_bottom]], dtype=np.int32)
    return vertices


class ObjectDetector(Detector):
    def __init__(self, agent: Agent, camera: Camera, name: str, **kwargs):
        super().__init__(agent, **kwargs)
        self.camera = camera
        self.prev_img = None
        self.name = name

    def run_in_series(self, **kwargs) -> Any:
        img = self.camera.data
        self.process_image_2(img, visualize=True)

    def run_in_threaded(self, **kwargs):
        pass

    def process_image_2(self, img, visualize=False):
        if img is None:
            return None

        processed_img = img.copy()

        left_vertices = get_vertices(img, 0, 0.30)
        left_mask = region_of_interest(left_vertices, np.zeros_like(img))

        right_vertices = get_vertices(img, 0.7, 1.0)
        right_mask = region_of_interest(right_vertices, np.zeros_like(img))

        wall_mask = cv2.bitwise_or(left_mask, right_mask)
        processed_img = self.detect_wall(processed_img, cv2.bitwise_and(img, wall_mask), visualize)

        mid_vertices = get_vertices(img, 0.30, 0.70)
        mid_mask = region_of_interest(mid_vertices, np.zeros_like(img))
        processed_img = self.detect_objects(processed_img, cv2.bitwise_and(img, mid_mask), visualize)

        if visualize:
            cv2.imshow("detection", processed_img)
            cv2.waitKey(1)

    def detect_wall(self, dst, img, visualize=False):
        gray_img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        blurred = cv2.GaussianBlur(gray_img, (7, 7), 0)

        thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 7, 2)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        valid_contours = []
        for cntr in contours:
            pt, size, angle = cv2.minAreaRect(cntr)
            x, y = pt
            w, h = size
            if (x + w < 240 or x + w > 560) and (y >= 250) and (h < 400) \
                    and (500 <= cv2.contourArea(cntr) < 2000):
                valid_contours.append(cntr)
                if visualize:
                    rect = cv2.minAreaRect(cntr)
                    box = np.int0(cv2.boxPoints(rect))
                    dst = cv2.drawContours(dst, [box], 0, (0, 0, 255), 2)

        if visualize:
            dst = cv2.drawContours(dst, valid_contours, -1, (0, 255, 0), 1)
        return dst

    def detect_objects(self, dst, image, visualize=False):
        if image is None:
            return None

        gray_img = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        blurred = cv2.GaussianBlur(gray_img, (7, 7), 0)

        thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 7, 2)
        contours, heirarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        valid_contours = []
        for cntr in contours:
            x, y, w, h = cv2.boundingRect(cntr)

            if (y >= 250) and (cv2.contourArea(cntr) >= 500):
                valid_contours.append(cntr)
                if visualize:
                    rect = cv2.minAreaRect(cntr)
                    box = np.int0(cv2.boxPoints(rect))
                    dst = cv2.drawContours(dst, [box], 0, (0, 0, 255), 2)

        if visualize:
            dst = cv2.drawContours(dst, valid_contours, -1, (0, 255, 0), 1)

        return dst

    def process_image(self, image, visualize=False, **kwargs):
        if image is None:
            return None

        gray_img = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

        if self.prev_img is None:
            self.prev_img = gray_img
            return None

        diff_image = cv2.absdiff(gray_img, self.prev_img)
        ret, thresh = cv2.threshold(diff_image, 100, 255, cv2.THRESH_BINARY)

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
