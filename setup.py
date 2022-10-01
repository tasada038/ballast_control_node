from setuptools import setup

package_name = 'ballast_control_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fish2',
    maintainer_email='fish2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pub_data = ballast_control_node.ballast_control_pub:main', 
            'ballast_control = ballast_control_node.ballast_control_node:main', 
            'joy_to_ballast = ballast_control_node.joy_to_ballast:main',
            'joy_to_topballast = ballast_control_node.joy_to_topballast:main',
            'joy_to_bottomballast = ballast_control_node.joy_to_bottomballast:main',
        ],
    },
)
