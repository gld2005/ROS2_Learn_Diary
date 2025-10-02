# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    前向位置控制器命令发布器启动文件
    这个文件启动一个节点，用于向forward_position_controller发送位置命令
    用于测试位置控制器的功能
    """

    # 找到位置目标配置文件路径
    # 这个YAML文件定义了要发送给控制器的位置目标值
    position_goals = PathJoinSubstitution(
        [
            FindPackageShare("ros2_control_demo_example_1"),
            "config",
            "rrbot_forward_position_publisher.yaml",  # 位置命令配置文件
        ]
    )

    # 返回启动描述
    return LaunchDescription(
        [
            Node(
                package="ros2_controllers_test_nodes",  # 测试节点包
                executable="publisher_forward_position_controller",  # 位置命令发布器可执行文件
                name="publisher_forward_position_controller",  # 节点名称
                parameters=[position_goals],  # 传入位置目标参数
                output="both",  # 输出到屏幕和日志
            )
        ]
    )