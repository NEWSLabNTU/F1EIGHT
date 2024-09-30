import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import yaml
from sensor_msgs.msg import CameraInfo, Image

def yaml_to_CameraInfo(yaml_fname):
    """
    Parse a yaml file containing camera calibration data into a 
    sensor_msgs/CameraInfo msg.
    
    Parameters
    ----------
    yaml_fname : str
        Path to yaml file containing camera calibration data
    Returns
    -------
    camera_info_msg : sensor_msgs.msg.CameraInfo
        A sensor_msgs.msg.CameraInfo message containing the camera calibration
        data
    """
    # Load data from file
    with open(yaml_fname, "r") as file_handle:
        calib_data = yaml.safe_load(file_handle)
    
    # Parse
    camera_info_msg = CameraInfo()
    camera_info_msg.header.frame_id = calib_data["camera_name"]
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.k = calib_data["camera_matrix"]["data"]
    camera_info_msg.d = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.r = calib_data["rectification_matrix"]["data"]
    camera_info_msg.p = calib_data["projection_matrix"]["data"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]
    return camera_info_msg

class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('camera_info_publisher')

        # Get parameters
        yaml_fname =  get_package_share_directory('calibrated_camera_publisher')+'/config/rgb_raw_info.yaml'
        image_topic = '/camera/zedxm/zed_node/rgb_raw/image_raw_color'

        # Parse yaml file
        self.camera_info_msg = yaml_to_CameraInfo(yaml_fname)
        self.image_msg = Image()

        # Create publisher
        self.info_publisher = self.create_publisher(CameraInfo, 'camera_info', 10)
        self.image_publisher = self.create_publisher(Image, 'image', 10)

        # Create subscriber with the provided topic
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_cb,
            20
        )

    def image_cb(self, msg):
        self.camera_info_msg.header = msg.header
        self.image_msg = msg

        self.info_publisher.publish(self.camera_info_msg)
        self.image_publisher.publish(self.image_msg)

def main(args=None):
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)

    # Create the node and spin
    node = CameraInfoPublisher()
    rclpy.spin(node)

    # Destroy the node explicitly (optional - otherwise it will be done automatically when the node is garbage collected)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
