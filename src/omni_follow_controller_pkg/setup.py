from setuptools import setup

package_name = 'omni_follow_controller_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', ['launch/virtual_circle_follow.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OpenAI',
    maintainer_email='oai@example.com',
    description='Virtual circular leader and omni follower controller using relative-state feedback linearization.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'virtual_circle_leader = omni_follow_controller_pkg.virtual_circle_leader:main',
            'omni_follower_controller = omni_follow_controller_pkg.omni_follower_controller:main',
        ],
    },
)
