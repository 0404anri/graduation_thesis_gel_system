from setuptools import find_packages, setup

package_name = 'to_gel'

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
    maintainer='neya',
    maintainer_email='rr0151si@ed.ritsumei.ac.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'task_manager = to_gel.task_manager:main',
            'vision_server = to_gel.vision_server:main',
            'yolov8_task_manager = to_gel.yolov8_task_manager:main',
            'yolov8_vision_server = to_gel.yolov8_vision_server:main',
        ],
    },
)
