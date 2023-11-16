from setuptools import setup

package_name = 'robot_daq'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sss0301',
    maintainer_email='atom.9031@gmail.com',
    description='Bunch of nodes for recording data',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_pid_daq = robot_daq.daq_depth_pid:main',
            'angle_pid_daq = robot_daq.daq_angle_pid:main',
            'daq_wheel_pid = robot_daq.daq_wheel_pid:main',
        ],
    },
)
