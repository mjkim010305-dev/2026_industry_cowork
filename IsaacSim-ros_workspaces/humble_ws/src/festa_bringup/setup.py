from glob import glob

from setuptools import setup

package_name = 'festa_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/bt', glob('bt/*.xml')),
        ('share/' + package_name + '/params', glob('params/*.yaml')),
        ('share/' + package_name + '/maps', glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mona',
    maintainer_email='yhc122833@gmail.com',
    description='One-shot bringup for the AI Festa L-course green-box sweep scenario.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'send_goal = festa_bringup.send_goal:main',
        ],
    },
)
