from setuptools import find_packages, setup

package_name = 'ur10e_custom_control'

setup(
    name=package_name,
    version='0.0.0',
    # packages=find_packages(exclude=['test']),
    packages = [package_name, f"{package_name}/python_utils/ros2_utils/comms"],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='georgekouretas',
    maintainer_email='georgekouretas@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_move = ur10e_custom_control.test_move:main',
            'sim_sensor = ur10e_custom_control.ur10e_simulated_sensor:main'
        ],
    },
)
