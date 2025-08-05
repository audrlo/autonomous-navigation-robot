from setuptools import find_packages, setup

package_name = 'roboclaw_driver'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robot Builder',
    maintainer_email='you@example.com',
    description='RoboClaw 2x30A motor controller driver for ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'roboclaw_node = roboclaw_driver.roboclaw_node:main',
        ],
    },
)