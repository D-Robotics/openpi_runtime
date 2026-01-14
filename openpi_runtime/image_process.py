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
        pad_value=(0, 0, 0),  # 填充的颜色，默认是黑色
    ):
        """
        target_size: (width, height)
        """
        self.target_width, self.target_height = target_size
        self.interpolation = interpolation or cv2.INTER_LINEAR
        self.pad_value = pad_value

    def process(self, img: np.ndarray) -> np.ndarray:
        """
        img: (H, W, 3), uint8
        return: (1, 3, 224, 224), uint8
        """
        if not isinstance(img, np.ndarray):
            raise TypeError("Input must be a numpy.ndarray")

        # 获取原始图像的宽度和高度
        original_height, original_width = img.shape[:2]

        # 计算等比例缩放的目标尺寸
        scale = self.target_width / original_width
        new_width = self.target_width
        new_height = int(original_height * scale)

        # 如果缩放后的高度大于目标高度，则根据目标高度重新计算缩放比例
        if new_height > self.target_height:
            scale = self.target_height / original_height
            new_height = self.target_height
            new_width = int(original_width * scale)

        # 进行等比例缩放
        img_resized = cv2.resize(img, (new_width, new_height), interpolation=self.interpolation)

        # 计算填充量
        top = (self.target_height - new_height) // 2
        bottom = self.target_height - new_height - top
        left = (self.target_width - new_width) // 2
        right = self.target_width - new_width - left

        # 使用填充，使图像变为目标大小
        img_padded = cv2.copyMakeBorder(
            img_resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=self.pad_value
        )

        # 返回处理后的图像，转为 NCHW 格式
        # 由于填充后的图像尺寸已经是 target_size，我们直接进行通道转换
        img_padded = img_padded.transpose(2, 0, 1)  # 转换为 (C, H, W)
        img_padded = np.expand_dims(img_padded, axis=0)  # 增加batch维度，变为 (1, C, H, W)

        return img_padded

    def __call__(self, img: np.ndarray) -> np.ndarray:
        """allow processor(img)"""
        return self.process(img)


# class ImageProcessor:
#     def __init__(
#         self,
#         target_size=(224, 224),
#         interpolation=None,
#     ):
#         """
#         target_size: (width, height)
#         """
#         from cv2 import INTER_LINEAR

#         self.resizer = ImageResizer(
#             target_size=target_size,
#             interpolation=interpolation or INTER_LINEAR,
#         )
#         self.layout_converter = HWC2NCHW()

#     def process(self, img: np.ndarray) -> np.ndarray:
#         """
#         img: (H, W, 3), uint8
#         return: (1, 3, 224, 224), uint8
#         """
#         if not isinstance(img, np.ndarray):
#             raise TypeError("Input must be a numpy.ndarray")

#         # step 1: resize
#         img = self.resizer(img)

#         # step 2: HWC -> NCHW
#         img = self.layout_converter(img)

#         return img

#     def __call__(self, img: np.ndarray) -> np.ndarray:
#         """allow processor(img)"""
#         return self.process(img)