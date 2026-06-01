from setuptools import find_packages, setup

package_name = 'r2_brain'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'py_trees', 'py_trees_ros', 'rich'],
    zip_safe=True,
    maintainer='awabelgubshawi',
    maintainer_email='awab011@hotmail.com',
    description='R2 mission behaviour tree (py_trees) for ABU Robocon 2026 — Kung Fu Quest.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'r2_brain        = r2_brain.parent:main',
            'r2_brain_tester = r2_brain.tester:main',
        ],
    },
)
