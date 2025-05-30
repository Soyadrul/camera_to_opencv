from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'image_saver'

setup(
    name=package_name,
    version='0.0.0',
    #packages=find_packages(exclude=['test']),
    packages=['image_saver', 'image_saver.open_waters', 'image_saver.open_waters.wasr', 'image_saver.open_waters.weights', 'launch', 'resource'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    package_data={
        # Include all files in weights folder
        package_name: ['image_saver/open_waters/weights/*.pth'],
    },
    include_package_data=True,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_saver = image_saver.image_saver_node:main',
        ],
    },
)
