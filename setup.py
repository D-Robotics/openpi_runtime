from setuptools import find_packages, setup

package_name = "openpi_runtime"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/" + "launch", ["launch/runtime.launch.py"]),
    ],
    package_data={
        "common.pi0_process": ["tokenizer.json", "tokenizer_config.json"],
    },
    # install_requires=['setuptools'],
    install_requires=[
        "setuptools",
        "piper_sdk",  # 添加pip依赖
        "alicia_d_sdk",  # 添加pip依赖
        "protobuf",  # Protocol Buffers 依赖（用于 msg_pb2）
        "tyro",  # 命令行参数解析库
    ],
    zip_safe=True,
    maintainer="zixi01.chen",
    maintainer_email="zixi01.chen@d-robotics.cc",
    description="openpi runtime package on RDK 600",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "aliciaD_node = openpi_runtime.aliciaD_node:main",
            "piper_node = openpi_runtime.piper_node:main",
            "s600_inference_node = inference.s600_inference_node:main",
        ],
    },
)
