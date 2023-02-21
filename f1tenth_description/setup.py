import os
from glob import glob
from setuptools import setup

package_name = 'f1tenth_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'gazebo'), glob('gazebo/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'sdf'), glob('sdf/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anyone',
    maintainer_email='chrisgraham908@gmail.com',
    description='Urdf, gazebo and all the other description stuff for the f1tenth',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
