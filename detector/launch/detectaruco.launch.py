"""Launch the USB camera node and aruco detector.

This launch file is intended show how the pieces come together.
Please copy the relevant pieces.

"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory as pkgdir

from launch                            import LaunchDescription
from launch.actions                    import Shutdown
from launch_ros.actions                import Node


#
# Generate the Launch Description
#
def generate_launch_description():

    ######################################################################
    # PREPARE THE LAUNCH ELEMENTS

    # Configure the USB camera node
    node_usbcam = Node(
        name       = 'usb_cam', 
        package    = 'usb_cam',
        executable = 'usb_cam_node_exe',
        namespace  = 'usb_cam',
        output     = 'screen',
        parameters = [{'camera_name':  'logitech'},
                      {'video_device': '/dev/video2'},
                      {'pixel_format': 'yuyv2rgb'},
                      {'image_width':  640},
                      {'image_height': 480},
                      {'framerate':           15.0},
                      {'brightness':          130},
                      {'contrast':            135},
                      {'saturation':          128},
                      {'sharpness':           -1},
                      {'gain':                -1},
                      {'white_balance_temperature':  5000},
                      {'white_balance_automatic': False},
                      {'autoexposure':        False},
                      {'exposure':            250},
                      {'autofocus':           True},
                      {'focus':               -1}])

    # Configure the aruco detector node
    node_aruco = Node(
        name       = 'aruco', 
        package    = 'detector',
        executable = 'detectaruco',
        output     = 'screen',
        remappings = [('/image_raw', '/usb_cam/image_raw')])


    ######################################################################
    # COMBINE THE ELEMENTS INTO ONE LIST
    
    # Return the description, built as a python list.
    return LaunchDescription([

        # Start the nodes.
        node_usbcam,
        node_aruco,
    ])
