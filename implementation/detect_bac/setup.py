from setuptools import setup

import os
from glob import glob

package_name = 'detect_bac'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='qq44642754',
    maintainer_email='qq44642754@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_node = detect_bac.detect_node:main',
            'cam_to_real = detect_bac.cam_to_real:main',
            'tf_broadcast = detect_bac.tf_broadcast:main',
            'tf_listener = detect_bac.tf_listener:main',
        ],
    },
)
