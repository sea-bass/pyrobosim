import os
from glob import glob
from setuptools import setup

package_name = 'pyrobosim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[
        package_name,
        package_name + "/gui",
        package_name + "/navigation",
        package_name + "/planning",
        package_name + "/utils",
        package_name + "/world"
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        # Include all YAML data files.
        (os.path.join('share', package_name, 'data'), 
         glob(os.path.join(package_name, 'data', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sebastian Castro',
    maintainer_email='sebas.a.castro@gmail.com',
    description='ROS2 enabled 2D mobile robot simulator for behavior prototyping.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_script = pyrobosim.test_script:main',
            'test_script_ros = pyrobosim.test_script:main_ros'
        ],
    },
)
