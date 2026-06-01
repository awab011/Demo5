from setuptools import find_packages, setup

package_name = 'r2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/r2_demo4.launch.py',
          'launch/test_r1Loc.launch.py',
          'launch/test_arena_tuning.launch.py',
          'launch/test_fusion.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='utmrbc',
    maintainer_email='awab011@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'kfs_camera_node        = r2.kfs_camera_detection:main',
            'kfs_lidar_node         = r2.kfs_lidar_detection:main',
            'kfs_fusion_node        = r2.lidar_camera_fusion:main',
            'r1_localization_node   = r2.r1_localization:main',
            'lidar_preprocess_node  = r2.lidar_preprocessing:main',
            'r2_localization_node   = r2.r2_localization:main',
        ],
    },
)
