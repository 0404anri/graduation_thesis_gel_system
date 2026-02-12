from setuptools import find_packages, setup

package_name = 'from_gel'

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
            'ina219_plotter = from_gel.ina219_plotter:main',
            'goal_generator = from_gel.goal_generator:main',
            'robot_controller = from_gel.robot_controller:main',
        ],
    },
)
