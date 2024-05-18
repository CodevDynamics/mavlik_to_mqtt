from setuptools import setup

setup(
    name='imu_serial',
    version='1.1.0',
    packages=['imu_serial'],
    install_requires=[
        'setuptools',
    ],
    scripts=[
        'src/lidar_mavlink_node_udp_livox.py',
        'src/mavlink.py',
    ],
    author='abner',
    author_email='huangwenfuture@sina.com',
    description='udp for livox and glio slam',
    license='BSD',
    keywords='ROS',
    classifiers=[
        'Programming Language :: Python',
        'Programming Language :: Python :: 2.7',
        'Programming Language :: Python :: 3.6',
    ],
)
