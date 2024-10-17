from setuptools import find_packages, setup

package_name = 'target_visualizer'

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
    maintainer='chenx',
    maintainer_email='chenx_dust@outlook.com',
    description='Target Visualizer',
    license='The MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'target_visualizer = target_visualizer.target_visualizer:main'
        ],
    },
)
