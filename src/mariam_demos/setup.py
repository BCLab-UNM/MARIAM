from setuptools import find_packages, setup

package_name = 'mariam_demos'

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
            'SineMoveBase = src.SineMoveBase:main'
        ],
    },
)
