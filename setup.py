from setuptools import find_packages, setup

package_name = 'bgt60tr13c_ros2'

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
    maintainer='lucas',
    maintainer_email='imzhangxiangbo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tr13c_node = bgt60tr13c_ros2.tr13c_node:main',
            'tr13c_listener = bgt60tr13c_ros2.tr13c_listener:main',
            'tr13c_node_listener = bgt60tr13c_ros2.tr13c_node_listener:main',
            'tr13c_testing = bgt60tr13c_ros2.tr13c_testing:main',
            'tr13c_plot = bgt60tr13c_ros2.tr13c_plot:main',
            'avian_ex = bgt60tr13c_ros2.avian_ex:main',
        ],
    },
)
