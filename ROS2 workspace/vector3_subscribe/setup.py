from setuptools import setup

package_name = 'vector3_subscribe'

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
    maintainer='begin',
    maintainer_email='begin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'subscriber = vector3_subscribe.vec:main',
            'pub = vector3_subscribe.vec_pub:main',
            'odometry = vector3_subscribe.odometry:main',
        ],
    },
)
