from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

import xacro
def concatenate_ns(ns1, ns2, absolute=False):
    
    if(len(ns1) == 0):
        return ns2
    if(len(ns2) == 0):
        return ns1
    
    # check for /s at the end and start
    if(ns1[0] == '/'):
        ns1 = ns1[1:]
    if(ns1[-1] == '/'):
        ns1 = ns1[:-1]
    if(ns2[0] == '/'):
        ns2 = ns2[1:]
    if(ns2[-1] == '/'):
        ns2 = ns2[:-1]
    if(absolute):
        ns1 = '/' + ns1
    return ns1 + '/' + ns2

def generate_launch_description():
    initial_positions_1_param = 'initial_positions_1'
    initial_positions_2_param = 'initial_positions_2'
    use_rviz_param = 'use_rviz'

    initial_positions_1 = LaunchConfiguration(initial_positions_1_param)
    initial_positions_2 = LaunchConfiguration(initial_positions_2_param)
    use_rviz = LaunchConfiguration(use_rviz_param)

    # Fixed values
    load_gripper = False # We make gripper a fixed variable, mainly because parsing the argument 
                        # within generate_launch_description is a fairly unintuitive process, 
                        # and it's not worth doing just for a single boolean.
    
    yaml_config = 'sim_garmi.yaml'
                        
    if(load_gripper): # mujoco scene file must be manually adjusted since there's no way to pass parameters
        scene_file = 'mob_manip.xml'
    else:
        scene_file = 'mob_manip.xml'
    garmi_xacro_file = os.path.join(get_package_share_directory('alberto_description'), 'robots','sim','summit_dual_panda',
                                     'mob_manip.urdf.xacro')
    xml_path = os.path.join(get_package_share_directory('alberto_description'), 'mujoco', 'summit_dual_panda', 'assets', 'xml', scene_file)
    mjros_config_file = os.path.join(get_package_share_directory('alberto_bringup'), 'config', 'sim',
                                     yaml_config)
    ##################################################################
    #   TPIK parametrs
    ###################################################################
    
    # TPIK_param_file = os.path.join(get_package_share_directory('garmi_controllers'), 'config', 'sim', 'TPIK_param.yaml')
    # declare_TPIK_param = DeclareLaunchArgument(
    #     'TPIK_param_file',
    #     default_value=TPIK_param_file,
    #     description='Path to the yaml file containing the TPIK parameters.'
    # )
    # load_TPIK_param = Node(
    #     package='rclcpp_components',
    #     executable='component_container',
    #     namespace='controller_manager',
    #     output='screen',
    #     parameters=[LaunchConfiguration('TPIK_param_file')],
    # )
  


    ####################################################################
    ############################################################################
    
    ns = ''     # this must match the namespace argument under mujoco_ros2_control in the plugin's parameter yaml file. 
                # See the ros2_control_plugins_example_with_ns.yaml file for more details.

    # Robot state publisher setup
    robot_description = Command(
        [FindExecutable(name='xacro'), ' ', garmi_xacro_file, 
            ' arm_id_1:=left', 
            ' arm_id_2:=right',
            ' hand_1:=', str(load_gripper).lower(),
            ' hand_2:=', str(load_gripper).lower(),
            ' initial_positions_1:=', initial_positions_1,
            ' initial_positions_2:=', initial_positions_2])
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace= ns,
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description}]
    )
    
    # Joint state publisher setup
    jsp_source_list = [concatenate_ns(ns, 'joint_states', True)]
    #if(load_gripper):
        #jsp_source_list.append(concatenate_ns(ns, 'left_gripper_sim_node/joint_states/joint_states', True))
        #jsp_source_list.append(concatenate_ns(ns, 'right_gripper_sim_node/joint_states/joint_states', True))

    node_joint_state_publisher = Node( # RVIZ dependency
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace= ns,
            parameters=[
                {'source_list': jsp_source_list,
                 'rate': 30}],
    )
    #############################################################################################
    #############################################################################################    
    node_robot_perception_right = Node(
        package='garmi_robot_perception',
        executable='proximity_task_generator',
        name='proximity_task_generator_right',
        namespace=ns,
        output='screen',
        parameters=[
            {'base_link_name': 'common_link'},
            {'load_gripper': load_gripper},
            {'arm_name': 'right'},
            {'other_arm': 'left'},
            {'use_sim_time': True},
            {'model_directory': "garmi_model"},
            {'urdf_string': "mob_manip.urdf"},
            {'has_arms': 2},
            {'armid_left': 'left'},
            {'armid_right': 'right'}
        ]
    )
    node_robot_perception_left = Node(
        package='garmi_robot_perception',
        executable='proximity_task_generator',
        name='proximity_task_generator_left',
        namespace=ns,
        output='screen',
        parameters=[
            {'base_link_name': 'common_link'},
            {'load_gripper': load_gripper},
            {'arm_name': 'left'},
            {'other_arm': 'right'},
            {'use_sim_time': True},
            {'model_directory': "garmi_model"},
            {'urdf_string': "mob_manip.urdf"},
            {'has_arms': 2},
            {'armid_left': 'left'},
            {'armid_right': 'right'}
        ]
    )
    node_robot_perception_summit = Node(
        package='garmi_robot_perception',
        executable='summit_proximity_task_generator',
        name='proximity_task_generator_summit',
        namespace=ns,
        output='screen'
    )
    node_cube_visualizer = Node(
        package='garmi_robot_perception',
        executable='cube_point_cloud',
        name='cube_visualizer',
        namespace=ns,
        output='screen',
        parameters=[
            {'base_link_name': 'common_link'},
            {'use_sim_time': True}
        ]
    )

    pcl_environment = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
           PathJoinSubstitution(
               [FindPackageShare('pointcloud_interactive_markers'), 'launch', 'cylinder_broadcast_launch.py']
           )
        )
    )
    #############################################################################################
    #############################################################################################
    # Others
    rviz_file = os.path.join(get_package_share_directory('garmi_description'), 'rviz',
                             'SummitPcl.rviz')
    

    return LaunchDescription([
        # Launch args
        DeclareLaunchArgument(
            use_rviz_param,
            default_value='false',
            description='Visualize the robot in Rviz'),
        DeclareLaunchArgument(
            initial_positions_1_param,
            #default_value='"0.0 -0.785 0.0 -2.356 0.0 1.571 0.785"',
            default_value='"0.0 -0.565 0.0 -2.2 0.0 1.571 0.785"',

            description='Initial joint positions of robot 1. Must be enclosed in quotes, and in pure number.'
                        'Defaults to the "communication_test" pose.'),
        DeclareLaunchArgument(
            initial_positions_2_param,
            default_value='"0.0 -0.785 0.0 -2.356 0.0 1.571 0.785"',
            description='Initial joint positions of robot 2. Must be enclosed in quotes, and in pure number.'
                        'Defaults to the "communication_test" pose.'),
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(get_package_share_directory('garmi_bringup') + '/launch/sim/launch_mujoco_ros_server.launch'),
            launch_arguments={
                'use_sim_time': "true",
                'modelfile': xml_path,
                'verbose': "true",
                'ns': ns,
                'mujoco_plugin_config': mjros_config_file

            }.items()
        ),

        
        #pcl_environment,
        node_cube_visualizer,



        node_robot_state_publisher,
        # node_joint_state_publisher,

        Node( # RVIZ dependency
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '-c', concatenate_ns(ns, 'controller_manager', True)],
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['--display-config', rviz_file],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(use_rviz)
             ),
        # Parameter loading
        # declare_TPIK_param,
        # load_TPIK_param,

        # Robot perception nodes
        node_robot_perception_right,
        node_robot_perception_left,
        node_robot_perception_summit


    ])
