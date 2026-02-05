from setuptools import find_packages, setup

package_name = "openpi_runtime"

setup(
    name=package_name,
    version="0.2.3",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    package_data={
        "common.pi0_process": ["tokenizer.json", "tokenizer_config.json"],
    },
    install_requires=[
        "setuptools",
        "piper_sdk",
        "alicia_d_sdk",
        "protobuf",
        "tyro",
    ],
    zip_safe=True,
    maintainer="zixi01.chen",
    maintainer_email="zixi01.chen@d-robotics.cc",
    description="openpi runtime package on RDK 600",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "aliciaD_node = teleop.aliciaD_node:main",
            "piper_node = robot.piper_node:main",
            "s600_inference_node = inference.s600_inference_node:main",
        ],
    },
)
