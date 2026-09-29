from setuptools import setup

package_name = 'green_box_approach'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/green_box_approach.launch.py']),
        ('share/' + package_name + '/config', ['config/green_box_approach_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mona',
    maintainer_email='yhc122833@gmail.com',
    description='HSV colour + LIDAR green box detector for the AI Festa demo (no object-recognition AI).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector_node = green_box_approach.detector_node:main',
        ],
    },
)
