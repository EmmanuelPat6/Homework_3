from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'marker_id',
            default_value='201',
            description='ID del marker ArUco da rilevare.'
        ),
        DeclareLaunchArgument(
            'marker_size',
            default_value='0.1',
            description='Dimensione del marker ArUco in metri.'
        ),
        DeclareLaunchArgument(
            'camera_topic',
            default_value='/videocamera',
            description='Topic della fotocamera da usare per il rilevamento.'
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera_info',
            description='Topic delle informazioni della fotocamera.'
        ),
        DeclareLaunchArgument(
            'marker_frame', 
            default_value='aruco_marker_frame',
            description='Frame ID associato al marker.'
        ),
        DeclareLaunchArgument(
            'reference_frame', 
            default_value='',
            description='Frame di riferimento per la posa calcolata.'
        ),
        DeclareLaunchArgument(
            'corner_refinement', 
            default_value='LINES',
            description='Metodo di affinamento degli angoli del marker.'
        ),
    ]

    # Node configuration
    aruco_single_node = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_single',
        output='screen',
        parameters=[{
            'image_is_rectified': True,
            'marker_size': LaunchConfiguration('marker_size'),
            'marker_id': LaunchConfiguration('marker_id'),
            'reference_frame': LaunchConfiguration('reference_frame'),
            'camera_frame': 'iiwa_camera_optical_frame',
            'marker_frame': LaunchConfiguration('marker_frame'),
            'corner_refinement': LaunchConfiguration('corner_refinement'),
        }],
        remappings=[
            ('/camera_info', LaunchConfiguration('camera_info_topic')),
            ('/image', LaunchConfiguration('camera_topic')),
        ]
    )

    # Combine arguments and node in LaunchDescription
    return LaunchDescription(declared_arguments + [aruco_single_node])
