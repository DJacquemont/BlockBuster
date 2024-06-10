import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'bb_state_machine'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config', 'mission1'), glob(os.path.join('config', 'mission1', '*'))),
        (os.path.join('share', package_name, 'config', 'mission2'), glob(os.path.join('config', 'mission2', '*'))),
        (os.path.join('share', package_name, 'config', 'mission3'), glob(os.path.join('config', 'mission3', '*'))),
        (os.path.join('share', package_name, 'config', 'mission4'), glob(os.path.join('config', 'mission4', '*'))),
        (os.path.join('share', package_name, 'config', 'general'), glob(os.path.join('config', 'general', '*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='djacquem',
    maintainer_email='jacquemont.dim@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bb_state_machine_node = bb_state_machine.bb_state_machine_node:main'
        ],
    },
)
