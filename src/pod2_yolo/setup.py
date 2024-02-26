from setuptools import find_packages, setup

package_name = 'pod2_yolo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='computing',
    maintainer_email='rakshitnar12@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'intel = pod2_yolo.model_input_node.main',
        'yolosub = pod2_yolo.yolov8_ros2_subscriber.main',
        'yolopt = pod2_yolo.yolov8_ros2_pt.main'
        ],
    },
)
