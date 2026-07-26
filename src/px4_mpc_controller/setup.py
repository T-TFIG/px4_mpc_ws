from setuptools import find_packages, setup

package_name = 'px4_mpc_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/mpc_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nathaphong_meng',
    maintainer_email='107681869+T-TFIG@users.noreply.github.com',
    description='Custom MPC offboard navigation controller for PX4 SITL',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offboard_demo_node = px4_mpc_controller.offboard_demo_node:main',
            'mpc_node = px4_mpc_controller.mpc_node:main',
            'trajectory_generator = px4_mpc_controller.trajectory_generator:main',
            'vehicle_pose_publisher = px4_mpc_controller.vehicle_pose_publisher:main',
        ],
    },
)
