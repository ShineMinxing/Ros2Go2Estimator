from setuptools import setup, find_packages

setup(
    name='message_handle',
    version='0.0.1',  # 明确版本号
    packages=find_packages(),  # 自动发现包结构
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/message_handle']),
        ('share/message_handle', ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='PointCloud to LaserScan converter',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'message_handle_node = message_handle.message_handle_node:main',
        ],
    },
)