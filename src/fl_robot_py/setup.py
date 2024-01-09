import os
from glob import glob
from setuptools import setup

package_name = 'fl_robot_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch'))),
         (os.path.join('share', package_name, 'config'), 
         glob(os.path.join('config', '*.yaml'))),
         (os.path.join('share', package_name, 'urdf'), 
         glob(os.path.join('urdf', '*.xacro'))),
         (os.path.join('share', package_name, 'rviz'), 
         glob(os.path.join('rviz', '*.rviz'))),
         (os.path.join('share', package_name, 'worlds'), 
         glob(os.path.join('worlds', '*.world'))),
         (os.path.join('share', package_name, 'meshes'), 
         glob(os.path.join('meshes', '*.dae'))),
         (os.path.join('share', package_name, 'textures'), 
         glob(os.path.join('textures', '*.png'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vishu',
    maintainer_email='viswanathan.m@gmail.com',
    description='Floor robot in py',
    license='Apcahe licence 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
#          'check_msg = fl_robot_py.check_msg01:main',
#          'move_it = fl_robot_py.move01:main',
          'drive_it = fl_robot_py.drive_fl_robot:main',
        ],
    },
)
