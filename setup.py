from setuptools import find_packages, setup
from setuptools import setup
from glob import glob
import os
package_name = 'my_xarm6_app'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, f'{package_name}.motion', f'{package_name}.vision', f'{package_name}.llm'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),
    ],
    install_requires=['setuptools', 'opencv-python', 'numpy', 'scipy', 'open3d', 'pyyaml', 'transforms3d', 'rosidl_default_generators', 'rosidl_default_runtime'],
    zip_safe=True,
    maintainer='fra',
    maintainer_email='ros.master.ai@gmail.com',
    description='Application layer for xArm6 (MoveIt + Vision)',
    license='Apache-2.0',
        extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'test_ik = my_xarm6_app.motion.test_ik:main',
            'move_to_pose = my_xarm6_app.motion.move_to_pose:main',
            'move_cnc_style = my_xarm6_app.motion.move_cnc_style:main',
            'move_cnc_style_circle = my_xarm6_app.motion.move_cnc_style_circle:main',
            'pick_place = my_xarm6_app.motion.pick_place:main',
            'color_detection = my_xarm6_app.vision.color_detection:main',
            'object_position = my_xarm6_app.vision.object_position:main',
            'object_position_server = my_xarm6_app.vision.object_position_server:main',
            'pick_place_vision = my_xarm6_app.motion.pick_place_vision:main',
            'move_to_position_llm = my_xarm6_app.motion.move_to_position_llm:main',
            'llm_command_node = my_xarm6_app.llm.llm_command_node:main',
            'llm_task_node = my_xarm6_app.llm.llm_task_node:main',
            'pick_place_vision_llm = my_xarm6_app.motion.pick_place_vision_llm:main',
            'general_llm_task = my_xarm6_app.llm.general_llm_task:main',
            'general_executor = my_xarm6_app.motion.general_executor:main',
        ],
    },
)

