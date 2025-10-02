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

# 从launch模块导入必要的类
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

# 从launch_ros模块导入ROS相关的类和函数
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    生成启动描述的主函数
    这个函数会被ROS2的launch系统调用，返回一个LaunchDescription对象
    """
    
    # 声明启动参数列表
    # 这些参数允许用户在启动时自定义行为
    declared_arguments = []
    
    # 声明描述文件包参数
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",  # 参数名称
            default_value="ros2_control_demo_description",  # 默认值
            description="包含机器人URDF/xacro文件的描述包。通常不设置此参数，\
                        以便使用自定义描述包。"  # 参数描述
        )
    )
    
    # 声明描述文件参数
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",  # 参数名称
            default_value="rrbot.urdf.xacro",  # 默认使用rrbot的xacro文件
            description="包含机器人描述的URDF/XACRO文件。"  # 参数描述
        )
    )
    
    # 声明GUI参数
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",  # 参数名称
            default_value="true",  # 默认启动GUI
            description="是否自动启动Rviz2和关节状态发布器GUI。\
                        true启动，false不启动。"  # 参数描述
        )
    )
    
    # 声明前缀参数（用于多机器人设置）
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",  # 参数名称
            default_value='""',  # 默认空字符串
            description="关节名称的前缀，对于多机器人设置很有用。\
                        如果更改此外，控制器配置中的关节名称也需要更新。"  # 参数描述
        )
    )

    # 初始化参数配置
    # 将启动参数转换为可在节点中使用的配置对象
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    gui = LaunchConfiguration("gui")
    prefix = LaunchConfiguration("prefix")

    # 通过xacro命令获取URDF描述
    # Command对象用于执行shell命令并获取输出
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),  # 查找xacro可执行文件路径
            " ",  # 空格分隔符
            PathJoinSubstitution(
                # 构建xacro文件的完整路径
                [FindPackageShare("ros2_control_demo_example_1"), "urdf", description_file]
            ),
            " ",  # 空格分隔符
            "prefix:=",  # 传递prefix参数给xacro文件
            prefix,  # prefix参数值
        ]
    )
    
    # 创建机器人描述参数字典
    # 这个字典将被传递给robot_state_publisher节点
    robot_description = {"robot_description": robot_description_content}

    # 构建RViz配置文件的路径
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rrbot/rviz", "rrbot.rviz"]
    )

    # 创建关节状态发布器节点
    # 这个节点提供一个GUI来手动控制机器人的关节
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",  # 包名
        executable="joint_state_publisher_gui",  # 可执行文件名称
        condition=IfCondition(gui),  # 条件：只有在gui参数为true时才启动
    )
    
    # 创建机器人状态发布器节点
    # 这个节点读取URDF描述，发布机器人的tf变换和关节状态
    robot_state_publisher_node = Node(
        package="robot_state_publisher",  # 包名
        executable="robot_state_publisher",  # 可执行文件名称
        output="both",  # 输出到屏幕和日志文件
        parameters=[robot_description],  # 传递机器人描述参数
    )
    
    # 创建RViz2节点
    # RViz是ROS的可视化工具，用于显示机器人模型、传感器数据等
    rviz_node = Node(
        package="rviz2",  # 包名
        executable="rviz2",  # 可执行文件名称
        name="rviz2",  # 节点名称
        output="log",  # 输出到日志文件
        arguments=["-d", rviz_config_file],  # 指定RViz配置文件
        condition=IfCondition(gui),  # 条件：只有在gui参数为true时才启动
    )

    # 将所有节点放入列表
    nodes = [
        joint_state_publisher_node,   # 关节状态发布器GUI
        robot_state_publisher_node,   # 机器人状态发布器
        rviz_node,                    # RViz可视化工具
    ]

    # 返回启动描述
    # 包含所有声明的参数和要启动的节点
    return LaunchDescription(declared_arguments + nodes)