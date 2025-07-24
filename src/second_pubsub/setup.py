from setuptools import find_packages, setup

package_name = 'second_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test'], include=['second_pubsub', 'second_pubsub.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='Package description',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'talker = second_pubsub.talker3:main',
		'listner = second_pubsub.listner3:main',
        'callibrate = second_pubsub.callibrate:main',
        ],
    },
)
