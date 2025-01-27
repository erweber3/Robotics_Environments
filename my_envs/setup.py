from setuptools import find_packages, setup

package_name = 'my_envs'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', [
    'launch/indoor_env_launch.py',
    'launch/outdoor_env_launch.py'
]))

data_files.append(('share/' + package_name + '/worlds', [
    'worlds/indoor_env.wbt',
    'worlds/outdoor_env.wbt',
]))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/' + package_name + '/resource', [
    'resource/turtlebot_webots.urdf',
    'resource/ros2control.yml',
]))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    maintainer='Monica Anderson-UA',
    maintainer_email='anderson@ua.edu',
    description='ROS2 package for Webots simulation with indoor and outdoor environments.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'launch.frontend.launch_extension': ['launch_ros = launch_ros'],    
        'console_scripts': ['my_envs = my_envs.my_envs:main']
    },
)
