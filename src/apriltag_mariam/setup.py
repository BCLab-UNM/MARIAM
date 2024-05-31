from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'apriltag_mariam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Include package folders
        (os.path.join('share', package_name, 'resource'), glob('resource/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'src'), glob('src/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='calvinjs',
    maintainer_email='cjstahoviak@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
