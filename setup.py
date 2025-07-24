from setuptools import find_packages, setup

package_name = 'minibot_base'

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
    maintainer='mini-ros2',
    maintainer_email='mini-ros2@todo.todo',
    description='Port from original Hypha-ROS minibot base driver on Virtuoso ROS1 minibot!',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'base_control = minibot_base.base_control:main',
        ],
    },
)
