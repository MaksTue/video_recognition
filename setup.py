from setuptools import setup

package_name = 'video_recognition'

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
    maintainer='maxim',
    maintainer_email='maxim@todo.todo',
    description='Traffic sign recognition node',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'recognition = video_recognition.classifier_node:main',
                'detection = video_recognition.detection_node:main',
                'speed_control = video_recognition.speed_control_node:main',
        ],
    },
)
