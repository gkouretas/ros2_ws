from setuptools import find_packages, setup
import glob
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
    ] + [tuple(['share/' + package_name, [x]]) for x in glob.glob("launch/*.py")],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='georgekouretas',
    maintainer_email='georgekouretas@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'home_robot = ur10e_custom_control.home_robot:main',
            'joint_velocity_test = ur10e_custom_control.joint_velocity_controller:main',
            'ur_control_qt = ur10e_custom_control.ur_control_qt:main'
        ],
    },
)
