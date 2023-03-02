from setuptools import setup

package_name = 'f1tenth_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anyone',
    maintainer_email='chrisgraham908@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'throttle_interpolator = f1tenth_control.throttle_interpolator:main',
            'environment = f1tenth_control.environment:main',
            'entity = f1tenth_control.entity:main',
        ],
    },
)
