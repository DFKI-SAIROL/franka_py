from setuptools import setup
import os
from glob import glob

package_name = 'franka_meta_quest'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='…',
    license='…',
    entry_points={
        'console_scripts': [
            'oculus_action_main = franka_meta_quest.oculus_action.main:main',
            'meta_quest_audio_publisher = franka_meta_quest.oculus_action.audio_publisher:main',
        ],
    },
)
