from setuptools import find_packages, setup

package_name = 'ros2_serial_arduino'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dhanush Shettigar',
    maintainer_email='dhanushshettigar90@gmail.com',
    description='ROS2 package for serial communication with arduino',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          "list_ports=ros2_serial_arduino.list_ports:main",
          "serial_reader=ros2_serial_arduino.serial_reader:main",
          "serial_writer=ros2_serial_arduino.serial_writer:main",
        ],
    },
)
