from setuptools import find_packages, setup

package_name = 'lab8_lidar'

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
    maintainer='t5',
    maintainer_email='c27Ramya.Ramesh@afacademy.af.edu',
    description='TODO: Package description',
    license='BSD-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'line_follower = lab8_lidar.line_follower:main',
        'wall_detector = lab8_lidar.wall_detector:main',
        ],
    },
)
