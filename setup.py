from setuptools import find_packages, setup

package_name = 'openpi_runtime'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/' + 'launch', ['launch/runtime.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zixi01.chen',
    maintainer_email='zixi01.chen@d-robotics.cc',
    description='openpi runtime package on RDK 600',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'openpi_runtime_node = openpi_runtime.openpi_runtime_node:main',
            'data_collection_node = openpi_runtime.data_collection_node:main',
        ],
    },
)
