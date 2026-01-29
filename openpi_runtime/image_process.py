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
        adapt_to_pi=True,
    ):
        """
        图像处理器，将图像调整为指定大小并转换为NCHW格式
        在adapt_to_pi=true配置下，还会进行额外的坐标转换和图像增强
        
        Args:
            target_size: 目标图像尺寸 (width, height)
            interpolation: 插值方法
            adapt_to_pi: 是否适配pi内部运行时
        """
        self.target_size = target_size
        self.interpolation = interpolation or cv2.INTER_LINEAR
        self.adapt_to_pi = adapt_to_pi
        self.resizer = ImageResizer(
            target_size=target_size,
            interpolation=self.interpolation
        )
        self.layout_converter = HWC2NCHW()

    def _coordinate_system_conversion(self, img: np.ndarray) -> np.ndarray:
        """
        坐标系统转换：将图像坐标系从标准Aloha系统转换为pi内部运行时使用的坐标系
        主要影响图像的旋转和镜像处理
        
        Args:
            img: 输入图像 (H, W, 3), uint8
            
        Returns:
            转换后的图像 (H, W, 3), uint8
        """
        # 镜像处理（根据pi内部运行时的要求）
        converted_img = cv2.flip(img, 1)  # 水平镜像
        
        return converted_img

    def _camera_parameter_adjustment(self, img: np.ndarray) -> np.ndarray:
        """
        相机参数调整：调整相机的内参和外参以匹配pi内部运行时的要求
        简单实现，实际项目中可能需要更复杂的相机参数调整
        
        Args:
            img: 输入图像 (H, W, 3), uint8
            
        Returns:
            调整后的图像 (H, W, 3), uint8
        """
        # 这里可以添加相机参数调整的代码
        # 由于没有具体的相机参数，这里暂时返回原始图像
        return img

    def _image_quality_enhancement(self, img: np.ndarray) -> np.ndarray:
        """
        图像质量增强：应用额外的图像增强算法以提高模型的识别准确率
        包括亮度调整、对比度增强和噪声去除
        
        Args:
            img: 输入图像 (H, W, 3), uint8
            
        Returns:
            增强后的图像 (H, W, 3), uint8
        """
        # 亮度和对比度增强
        alpha = 1.1  # 对比度调整因子
        beta = 10   # 亮度调整因子
        enhanced_img = cv2.convertScaleAbs(img, alpha=alpha, beta=beta)
        
        # 噪声去除
        enhanced_img = cv2.GaussianBlur(enhanced_img, (3, 3), 0)
        
        return enhanced_img

    def process(self, img: np.ndarray) -> np.ndarray:
        """
        处理图像，将其调整为目标大小并转换为NCHW格式
        在adapt_to_pi=true配置下，还会进行额外的坐标转换和图像增强
        
        Args:
            img: 输入图像 (H, W, 3), uint8
            
        Returns:
            处理后的图像 (1, C, H, W), uint8
        """
        if not isinstance(img, np.ndarray):
            raise TypeError("Input must be a numpy.ndarray")
            
        # 如果适配pi内部运行时，进行额外的图像处理
        if self.adapt_to_pi:
            # 1. 坐标系统转换
            img = self._coordinate_system_conversion(img)
            
            # 2. 相机参数调整
            img = self._camera_parameter_adjustment(img)
            
            # 3. 图像质量增强
            img = self._image_quality_enhancement(img)
        
        # 调整图像大小
        resized_img = self.resizer(img)
        
        # 转换为NCHW格式
        nchw_img = self.layout_converter(resized_img)
        
        return nchw_img

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