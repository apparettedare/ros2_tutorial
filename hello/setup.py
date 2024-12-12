from setuptools import setup
import os
from glob import glob

package_name = 'hello'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='apparettedare',
    maintainer_email='ksuzuki9541@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "hello_node = hello.hello_node:main",
            "talker = hello.talker:main",
            "listener = hello.listener:main",
            "client = hello.client:main",
            "server = hello.server:main",
            "action_client = hello.action_client:main",
            "action_server = hello.action_server:main",
        ],
    },
)
