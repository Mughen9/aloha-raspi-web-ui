import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
import subprocess
from aloha_msgs.srv import AutoRecord

class NodeManagerNode(Node):
    """Server node that manages ROS 2 bringup, sleep, and auto bash SSH."""
    
    def __init__(self):
        super().__init__("node_manager_node")
        self.get_logger().info("Initializing node_manager_node")

        # Services
        self._launch_ros2_srv = self.create_service(Trigger, "launch_ros2", self._launch_ros2_callback)
        self._run_sleep_srv = self.create_service(Trigger, "run_sleep", self._run_sleep_callback)
        self._run_auto_record_srv = self.create_service(AutoRecord, "run_auto_record", self._run_auto_record_callback)

        # Subscribe to camera topics from bringup
        self._cam_high_sub = self.create_subscription(Image, "/cam_high/camera/color/image_rect_raw", self._camera_callback, 10)
        self._cam_left_wrist_sub = self.create_subscription(Image, "/cam_left_wrist/camera/color/image_rect_raw", self._camera_callback, 10)
        self._cam_low_sub = self.create_subscription(Image, "/cam_low/camera/color/image_rect_raw", self._camera_callback, 10)
        self._cam_right_wrist_sub = self.create_subscription(Image, "/cam_right_wrist/camera/color/image_rect_raw", self._camera_callback, 10)

        # Process management
        self.bringup_process = None
        self.auto_record_process = None
        self.sleep_process = None

    def _camera_callback(self, msg):
        """Handle incoming camera images (can be processed if needed)."""
        

    def _launch_ros2_callback(self, request, response):
        """Launch ROS 2 bringup."""
        self.get_logger().info("Launching ROS 2 bringup...")
        try:
            launch_command = ["ros2", "launch", "aloha", "aloha_bringup.launch.py"]
            self.bringup_process = subprocess.Popen(launch_command)
            response.success = True
            response.message = "ROS 2 bringup launched successfully. Cameras should be publishing."
        except Exception as e:
            response.success = False
            response.message = f"Error launching ROS 2 bringup: {str(e)}"
        return response

    def _run_sleep_callback(self, request, response):
        """Run the sleep program."""
        self.get_logger().info("Preparing to run sleep program...")
        try:
            self._stop_ongoing_processes()
            sleep_command = ["python3", "/home/sai/interbotix_ws/src/aloha/scripts/sleep.py"]
            self.sleep_process = subprocess.Popen(sleep_command)
            response.success = True
            response.message = "Sleep program started after stopping other processes."
        except Exception as e:
            response.success = False
            response.message = f"Error running sleep program: {str(e)}"
        return response

    def _run_auto_record_callback(self, request, response):
        """Start auto record with task and episode count."""
        try:
            task_name = request.task_name
            num_episodes = request.num_episodes
            self.get_logger().info(f"Starting auto record for task {task_name} with {num_episodes} episodes.")
            valid_tasks = ['cutlery', 'aloha_mobile_hello_aloha']
            if task_name not in valid_tasks:
                response.success = False
                response.message = f"Task '{task_name}' is not valid."
                return response
            command = f"bash /home/sai/interbotix_ws/src/aloha/scripts/auto_record.sh {task_name} {num_episodes}"
            self.auto_record_process = subprocess.Popen(command, shell=True)
            response.success = True
            response.message = f"Auto record started for task {task_name} with {num_episodes} episodes."
        except Exception as e:
            response.success = False
            response.message = f"Error starting auto record: {str(e)}"
        return response

    def _stop_ongoing_processes(self):
        """Terminate ongoing processes except ROS 2 bringup."""
        if self.auto_record_process:
            self.get_logger().info("Stopping auto-record process...")
            self.auto_record_process.terminate()
            self.auto_record_process.wait()
            self.auto_record_process = None
        if self.sleep_process:
            self.get_logger().info("Terminating sleep process...")
            self.sleep_process.terminate()
            self.sleep_process.wait()
            self.sleep_process = None
        self.get_logger().info("All ongoing processes terminated except bringup.")

    def destroy_node(self):
        """Override destroy_node for cleanup."""
        self._stop_ongoing_processes()
        super().destroy_node()


def main():
    """Main function to initialize and spin the ROS 2 node."""
    rclpy.init()
    node = NodeManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
