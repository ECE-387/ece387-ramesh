from setuptools import find_packages, setup

package_name = 'lab11_apriltag'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    	('share/' + package_name + '/config', ['config/default_cam.yaml']), 
    	('share/' + package_name + '/config', ['launch/vision.launch.py']), 
    	('share/' + package_name + '/config', ['config/stop_detector.svm']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='t5',
    maintainer_email='c27Ramya.Ramesh@afacademy.af.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'apriltag_node = lab11_apriltag.apriltag:main',
        ],
    },
)
