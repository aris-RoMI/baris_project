from setuptools import find_packages, setup
import glob

package_name = 'baris_UI'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/ui/', glob.glob('src/' + package_name + '/ui/*.ui')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joe',
    maintainer_email='dlwlgh0106@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'baris_ui = baris_UI.baris_ui:main',
            'robot_control_service = baris_UI.robot_control_service:main',
            "robot_service_client = baris_UI.RobotServiceClient:main",
        ],
    },
)