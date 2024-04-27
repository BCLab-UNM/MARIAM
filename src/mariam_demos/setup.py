import os
from glob import glob
from setuptools import find_packages, setup
import os

package_name = 'mariam_demos'

# Function to get all files in a directory
def get_package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            # Correctly join the path and filename
            paths.append(os.path.join(path, filename))
    return paths

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
        data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/msg', ['msg/Int16Stamped.msg']),
        # Correctly specify the resource and launch files
        (os.path.join('share', package_name, 'resource'), get_package_files('resource')),
        (os.path.join('share', package_name, 'launch'), get_package_files('launch')),
        (os.path.join('share', package_name, 'src'), get_package_files('src'))
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='carter',
    maintainer_email='git@carterfrost.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dataPlot = src.dataPlot:main',
            'dataPlotKeepArm = src.dataPlotKeepArm:main',
            'DelayJoy = src.DelayJoy:main',
            'DelayJoyInvert = src.DelayJoyInvert:main',
            'JointEffortMoveBase = src.JointEffortMoveBase:main',
            'JoyMoveBase = src.JoyMoveBase:main',
            'offset = src.offset:main',
            'offsetMoveBase = src.offsetMoveBase:main',
            'overlay = src.overlay:main',
            'SineMoveBase = src.SineMoveBase:main',
            'Distance = src.distance:main',
            'JoyToCmdVel = src.JoyToCmdVel:main'
        ],
    },
)
