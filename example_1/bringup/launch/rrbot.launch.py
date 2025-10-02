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

"""
ROS2 Control 完整系统启动文件教学说明

这个launch文件启动一个完整的ROS2控制系统，包含以下核心组件：
1. ros2_control_node - 核心控制节点，管理硬件接口和控制器
2. 控制器管理器 - 动态加载和卸载控制器
3. 机器人状态发布器 - 发布机器人TF和状态信息
4. 关节状态广播器 - 发布关节状态数据
5. 位置控制器 - 执行位置控制指令
6. RViz2 - 可视化工具

系统启动顺序：
1. 首先启动ros2_control_node和机器人状态发布器
2. 然后启动位置控制器
3. 等待位置控制器启动完成后启动关节状态广播器
4. 最后启动RViz2可视化

数据流：控制指令 → 位置控制器 → 硬件接口 → 关节状态 → 状态广播器 → RViz显示
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    ROS 2 机器人控制系统完整启动文件
    这个文件启动了一个完整的ros2_control演示系统，包括：
    1. 机器人状态发布
    2. 控制器管理器
    3. 关节状态广播器
    4. 位置控制器
    5. RViz2可视化工具
    
    整个系统启动顺序经过精心设计，确保依赖关系正确
    """
    
    # 声明启动参数 - 这些参数可以在启动时从命令行覆盖
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",  # 参数名称
            default_value="true",  # 默认值
            description="Start RViz2 automatically with this launch file.",  # 描述
        )
    )
    '''
# 使用默认值启动（启动RViz2）
ros2 launch my_package robot.launch.py

# 显式启动RViz2
ros2 launch my_package robot.launch.py gui:=true

# 不启动RViz2  
ros2 launch my_package robot.launch.py gui:=false'''



    # 初始化参数 - 将声明的参数转换为可在节点中使用的配置对象
    gui = LaunchConfiguration("gui")

    # 通过xacro文件获取机器人URDF描述
    # 使用Command和xacro工具动态生成URDF文件内容
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),  # 找到xacro可执行文件路径， 执行结果: 返回xacro可执行文件的完整路径，如: /opt/ros/humble/bin/xacro

            " ",  # 空格分隔符- 在命令中分隔可执行文件和参数
            
            PathJoinSubstitution(  # 找到URDF xacro文件路径
                [
                    FindPackageShare("ros2_control_demo_example_1"),  # 查找软件包
                    "urdf",  # 包内路径 
                    "rrbot.urdf.xacro",  # xacro文件名
                ]
            ),

    ])
         # 执行结果: 返回xacro文件的完整路径，如: /workspace/src/ros2_control_demo_example_1/urdf/rrbot.urdf.xacro

    ''' # Command 用于执行shell命令并获取输出
            robot_description_content = Command([命令, 参数1, 参数2, ...]) #空格也算参数
            # 相当于在终端执行: xacro /path/to/rrbot.urdf.xacro
        '''
   
    # 创建机器人描述参数字典，这是robot_state_publisher需要的格式
    robot_description = {"robot_description": robot_description_content}

    # 找到控制器配置文件路径
    # 这个YAML文件定义了要加载的控制器及其参数
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("ros2_control_demo_example_1"),
            "config", 
            "rrbot_controllers.yaml",  # 控制器配置文件
        ]
    )
    
    # 找到RViz配置文件路径
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ros2_control_demo_description"), "rrbot/rviz", "rrbot.rviz"]
    )

    # 创建ROS 2控制节点 - 这是控制系统的核心
    # ros2_control_node是控制器管理器，负责加载和管理所有控制器
    control_node = Node(
        package="controller_manager",  # 控制器管理包
        executable="ros2_control_node",  # 控制器管理器可执行文件
        parameters=[robot_controllers],  # 传入控制器配置参数
        output="both",  # 输出到屏幕和日志文件
    )
    
    # 创建机器人状态发布器节点
    # 这个节点读取URDF和关节状态，发布机器人整体的TF变换和状态
    robot_state_pub_node = Node(
        package="robot_state_publisher",  # 状态发布器包
        executable="robot_state_publisher",  # 状态发布器可执行文件
        output="both",  # 输出到屏幕和日志文件
        parameters=[robot_description],  # 传入机器人描述参数
    )
    
    # 创建RViz2可视化节点
    # RViz2用于可视化机器人模型和状态，有条件启动（根据gui参数）
    rviz_node = Node(
        package="rviz2",  # RViz2包
        executable="rviz2",  # RViz2可执行文件
        name="rviz2",  # 节点名称
        output="log",  # 输出到日志
        arguments=["-d", rviz_config_file],  # 指定RViz配置文件
        condition=IfCondition(gui),  # 条件启动：只有当gui=true时才启动
    )

    # 创建关节状态广播器生成器节点
    # spawner是一个工具，用于动态加载和启动控制器
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",  # 控制器管理包
        executable="spawner",  # spawner可执行文件，用于启动控制器
        arguments=["joint_state_broadcaster"],  # 要启动的控制器名称
        # 注意：这里没有直接包含在nodes列表中，而是通过事件处理器控制启动时机
    )

    # 创建机器人位置控制器生成器节点
    # 这个控制器负责控制机器人的位置
    robot_controller_spawner = Node(
        package="controller_manager",  # 控制器管理包
        executable="spawner",  # spawner可执行文件
        arguments=[
            "forward_position_controller",  # 前向位置控制器名称
            "--param-file", robot_controllers  # 指定参数文件
        ],
    )

    # ==========================================================================
    # 启动顺序管理 - 这是确保系统正确启动的关键部分
    # ==========================================================================

    # 延迟RViz2启动，直到joint_state_broadcaster启动完成
    # 这是因为RViz需要关节状态数据才能正确显示机器人
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,  # 目标动作：关节状态广播器
            on_exit=[rviz_node],  # 当目标动作退出时执行的动作：启动RViz
        )
    )

    # 延迟joint_state_broadcaster启动，直到robot_controller启动完成
    # 注意：注释中提到这是一个临时解决方案，用于解决测试中的不稳定问题
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,  # 目标动作：机器人控制器
            on_exit=[joint_state_broadcaster_spawner],  # 当目标退出时：启动关节状态广播器
        )
    )

    # 组合所有节点和事件处理器
    nodes = [
        control_node,           # 1. 首先启动控制器管理器
        robot_state_pub_node,   # 2. 启动机器人状态发布器
        robot_controller_spawner,  # 3. 启动机器人控制器
        # 事件处理器会自动管理后续的启动顺序：
        # 4. 机器人控制器启动完成后 → 启动关节状态广播器
        # 5. 关节状态广播器启动完成后 → 启动RViz2
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
    ]

    # 返回启动描述，包含所有参数和节点
    return LaunchDescription(declared_arguments + nodes)