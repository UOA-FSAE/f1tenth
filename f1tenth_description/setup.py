import os
from glob import glob
from setuptools import setup

package_name = 'f1tenth_description'

def package_files(directory):
    """
    Package all data files recursively in a directory
    """
    paths = []
    for (path, _, filenames) in os.walk(directory):
        if len(filenames) > 0:
            paths.append((os.path.join('share', package_name, path), 
                          [os.path.join(path, filename) for filename in filenames]))
    return paths

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
        *package_files('sdf'),
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
