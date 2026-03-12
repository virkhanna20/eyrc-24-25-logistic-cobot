from setuptools import find_packages, setup

package_name = 'tf_broadcaster_pkg'

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
    maintainer='parthkhanna',
    maintainer_email='parthkhannawork@gmail.com',
    description='TF2 transform broadcasting and listening utilities',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'fixed_frame_broadcaster = tf_broadcaster_pkg.fixed_frame_broadcaster:main',
        'frame_listener = tf_broadcaster_pkg.frame_listener:main'
    ],
},

)
