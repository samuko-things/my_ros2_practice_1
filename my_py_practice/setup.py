from setuptools import find_packages, setup

package_name = 'my_py_practice'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    # data_files=[
    #     ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    #     ('share/' + package_name, ['package.xml']),
    #     (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    # ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='samuko',
    maintainer_email='samuel.c.agba@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          # 'executable_name = package_name.python_file_name:main'
          'robot_news_station = my_py_practice.robot_news_station:main',
          'smart_phone = my_py_practice.smart_phone:main',
          'add_two_ints_server = my_py_practice.add_two_ints_server:main',
          'add_two_ints_client_no_oop = my_py_practice.add_two_ints_client_no_oop:main',
          'add_two_ints_client = my_py_practice.add_two_ints_client:main',
          'hw_status_publisher = my_py_practice.hw_status_publisher:main',
          'number_counter = my_py_practice.number_counter:main',
          'number_publisher = my_py_practice.number_publisher:main'
        ],
    },
)
