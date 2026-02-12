from setuptools import find_packages, setup

package_name = 'rpi_controller'

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
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'gel_controller = to_from_gel.gel_controller:main',
        'gel_controller_yolov8 = to_from_gel.gel_controller_yolov8:main',
        ],
    },
)
