from setuptools import find_packages, setup

package_name = 'py_pubsub'

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
    maintainer='sahan',
    maintainer_email='sahan@robocon.uom',
    description='Simple PubSub Package v1.0',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'talker = py_pubsub.publisher_member_function:main',
        	'listner = py_pubsub.subscriber_member_function:main',
    	],
	},
)
