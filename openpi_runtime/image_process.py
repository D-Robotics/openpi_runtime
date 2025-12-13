# Copyright (c) 2025，D-Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import cv2
import numpy as np

class ImageResizer:
    def __init__(self, target_size=(224, 224), interpolation=cv2.INTER_LINEAR):
        """
        target_size: (width, height)
        """
        self.target_size = target_size
        self.interpolation = interpolation

    def __call__(self, img: np.ndarray) -> np.ndarray:
        """
        img: (H, W, C), uint8
        return: (224, 224, C), uint8
        """
        if img.ndim != 3 or img.shape[2] != 3:
            raise ValueError(f"Expect HWC RGB image, got shape={img.shape}")

        resized = cv2.resize(
            img,
            self.target_size,
            interpolation=self.interpolation,
        )
        return resized

class HWC2NCHW:
    def __call__(self, img: np.ndarray) -> np.ndarray:
        """
        img: (H, W, C), uint8
        return: (1, C, H, W), uint8
        """
        if img.ndim != 3 or img.shape[2] != 3:
            raise ValueError(f"Expect HWC RGB image, got shape={img.shape}")

        # HWC -> CHW
        chw = np.transpose(img, (2, 0, 1))

        # CHW -> NCHW
        nchw = np.expand_dims(chw, axis=0)

        return nchw.astype(np.uint8)

class ImageProcessor:
    def __init__(
        self,
        target_size=(224, 224),
        interpolation=None,
    ):
        """
        target_size: (width, height)
        """
        from cv2 import INTER_LINEAR

        self.resizer = ImageResizer(
            target_size=target_size,
            interpolation=interpolation or INTER_LINEAR,
        )
        self.layout_converter = HWC2NCHW()

    def process(self, img: np.ndarray) -> np.ndarray:
        """
        img: (H, W, 3), uint8
        return: (1, 3, 224, 224), uint8
        """
        if not isinstance(img, np.ndarray):
            raise TypeError("Input must be a numpy.ndarray")

        # step 1: resize
        img = self.resizer(img)

        # step 2: HWC -> NCHW
        img = self.layout_converter(img)

        return img

    def __call__(self, img: np.ndarray) -> np.ndarray:
        """allow processor(img)"""
        return self.process(img)