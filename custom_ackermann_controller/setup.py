from setuptools import find_packages, setup

package_name = 'custom_ackermann_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/robot_localization.yaml',
            'config/laser_scan_matcher.yaml',
            'config/smooth_velocity_controller.yaml'
        ]),
        ('share/' + package_name + '/launch', [
            'launch/joystick_teleop.launch.py',
            'launch/enhanced_odometry.launch.py',
            'launch/smooth_teleop.launch.py'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aditya-pachauri',
    maintainer_email='adi.pachauri.444@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ackermann_twist_controller = custom_ackermann_controller.ackermann_twist_controller:main',
            'wheel_odometry = custom_ackermann_controller.wheel_odometry:main',
            'laser_scan_matcher = custom_ackermann_controller.laser_scan_matcher:main',
            'smooth_velocity_controller = custom_ackermann_controller.smooth_velocity_controller:main',
        ],
    },
)
