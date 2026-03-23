from glob import glob
from setuptools import setup


package_name = 'omni_formation_controller_pkg'


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OpenAI',
    maintainer_email='oai@example.com',
    description='Leader-follower formation controller and two-robot bringup launches for Nexus omni bases.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'leader_follower_controller = omni_formation_controller_pkg.leader_follower_controller:main',
        ],
    },
)
