from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mc6'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[yma]*'))),
    (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
    (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*.stl'))),
    (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='israels',
    maintainer_email='isra.sanchez204@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'puzzlebot_sim = mc6.puzzlebot_sim:main',
            'controller = mc6.controller:main',
            'localisation = mc6.localisation:main',
            'point_generator = mc6.point_generator:main',
            'wall_following_bug0 = mc6.wall_following_bug0:main',
            'wall_following_bug2 = mc6.wall_following_bug2:main',
        ],
    },
)
