from setuptools import find_packages, setup

package_name = 'mocap_drone_interface'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools>=80.0.0'],
    zip_safe=True,
    maintainer='optimalx',
    maintainer_email='benfox0621@gmail.com',
    description='Takes motive data over wifi and creates Ros nodes that send position and controls for UAV autopilots.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = mocap_drone_interface.ros2tests.testpub:main',
            'listener = mocap_drone_interface.ros2tests.testsub:main',
            'posevis = mocap_drone_interface.ros2tests.rviz_mocap:main',
            'service = mocap_drone_interface.library.flightclient:main'
            
        ],
    },
)
