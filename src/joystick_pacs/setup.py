from setuptools import find_packages, setup

package_name = 'joystick_pacs'

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
    maintainer='zappington',
    maintainer_email='abhinavs1306@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibration_node = joystick_pacs.joystick_calibration_node:main',
            'joy_node = joystick_pacs.joyPublisher:main',
            'serialPub = joystick_pacs.serialPublisher:main',
            'armPub = joystick_pacs.armPublisher:main',
            'controlled_pub = joystick_pacs.controlled_publisher:main',
            'imu_pub = joystick_pacs.imu_publisher:main',
            'simpleSerialPub = joystick_pacs.simple_serial_publisher:main',
            'arm_gui = joystick_pacs.arm_button:main',
            'better_imu = joystick_pacs.imu_pub_updated:main',
            'param_gui = joystick_pacs.param_gui:main',
            'slider_gui = joystick_pacs.slider_gui:main',
            'manipulatorPublisher = joystick_pacs.manipulatorPublisher:main',
            'light_slider_gui = joystick_pacs.slider_gui_lights:main',
            'cmd_translator = joystick_pacs.cmd_translator:main',
	    'thruster_mixer = joystick_pacs.thruster_mixer:main',
	    'imu_calibrator = joystick_pacs.imu_calibrator:main',
	    'auv_test = joystick_pacs.sensor_auto_move:main',
        ],
    },
)
