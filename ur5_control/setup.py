from setuptools import find_packages, setup

package_name = 'ur5_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, [package_name + '/__init__.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Parth Khanna',
    maintainer_email='parthkhannawork@gmail.com',
    description='UR5 arm control with ArUco detection and TF publishing',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "aruco_detector = ur5_control.aruco_detector:main",
        ],
    },
)
