import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'perception_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        # 9단계: sample_data/fake_input.json을 패키지 안에 함께 담아서 설치한다.
        # colcon build/install 이후에는 소스 트리 상대경로(../sample_data/...)로
        # 파일을 찾을 수 없기 때문에(설치 위치가 소스 위치와 달라짐), 패키지
        # 공유 디렉터리(share/perception_pkg/sample_data/)에 복사해두고
        # ament_index_python으로 그 경로를 찾는다 (fake_d555_publisher.py,
        # detected_object_node.py 참고).
        (os.path.join('share', package_name, 'sample_data'), glob('sample_data/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team1',
    maintainer_email='you@example.com',
    description=(
        'TurtleBot3+D555+OpenMANIPULATOR-X 프로젝트 Perception(팀원1) 정식 ROS2 패키지 '
        '(Phase 1, 가짜 데이터 단계).'
    ),
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_d555_publisher = perception_pkg.fake_d555_publisher:main',
            'detected_object_node = perception_pkg.detected_object_node:main',
        ],
    },
)
