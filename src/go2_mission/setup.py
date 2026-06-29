from setuptools import find_packages, setup

package_name = 'go2_mission'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeonbotdae',
    maintainer_email='vkrtp2@gmail.com',
    description='GO2 EDU ICROS 2026 대회 패키지',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'diagnostics_subscriber = go2_mission.diagnostics_subscriber:main',
            'mission_manager = go2_mission.mission_manager:main',
            'toplight_detector = go2_mission.toplight_detector:main',
            'hsv_tuner = go2_mission.hsv_tuner:main',
        ],
    },
)
