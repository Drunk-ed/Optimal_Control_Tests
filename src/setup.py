from setuptools import find_packages, setup

package_name = 'mujoco_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='drunk',
    maintainer_email='gidijalaraja@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_state_pub = mujoco_test.joint_state_pub:main',
            'joint_state_sub = mujoco_test.joint_state_sub:main',
        ],
    },
)
