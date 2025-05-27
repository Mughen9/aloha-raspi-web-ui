import os
import json
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
import subprocess
from aloha_msgs.srv import AutoRecord

class NodeManagerNode(Node):
    def __init__(self):
        super().__init__("node_manager_node")
        self.get_logger().info("Initializing node_manager_node")

        # Load config
        self.scripts_dir = os.getenv("ALOHA_SCRIPT_DIR", "/home/user/workspace/src/aloha/scripts")
        self.task_config_path = os.path.join(self.scripts_dir, "task_config.json")
        self.valid_tasks = self._load_valid_tasks()

        # ROS Services
        self._launch_ros2_srv = self.create_service(Trigger, "launch_ros2", self._launch_ros2_callback)
        self._run_sleep_srv = self.create_service(Trigger, "run_sleep", self._run_sleep_callback)
        self._run_auto_record_srv = self.create_service(AutoRecord, "run_auto_record", self._run_auto_record_callback)

        # Camera Subscriptions
        self._cam_high_sub = self.create_subscription(Image, "/cam_high/camera/color/image_rect_raw", self._camera_callback, 10)
        self._cam_left_wrist_sub = self.create_subscription(Image, "/cam_left_wrist/camera/color/image_rect_raw", self._camera_callback, 10)
        self._cam_low_sub = self.create_subscription(Image, "/cam_low/camera/color/image_rect_raw", self._camera_callback, 10)
        self._cam_right_wrist_sub = self.create_subscription(Image, "/cam_right_wrist/camera/color/image_rect_raw", self._camera_callback, 10)

        # Process handles
        self.bringup_process = None
        self.auto_record_process = None
        self.sleep_process = None

    def _load_valid_tasks(self):
        if os.path.exists(self.task_config_path):
            with open(self.task_config_path, 'r') as f:
                try:
                    config = json.load(f)
                    return config.get("valid_tasks", [])
                except json.JSONDecodeError:
                    self.get_logger().warn("Failed to parse task_config.json.")
        return []

    def _camera_callback(self, msg):
        pass  # Optional: Add image processing if needed

    def _launch_ros2_callback(self, request, response):
        self.get_logger().info("Launching ROS 2 bringup...")
        try:
            launch_command = ["ros2", "launch", "aloha", "aloha_bringup.launch.py"]
            self.bringup_process = subprocess.Popen(launch_command)
            response.success = True
            response.message = "ROS 2 bringup launched successfully."
        except Exception as e:
            response.success = False
            response.message = f"Error launching bringup: {str(e)}"
        return response

    def _run_sleep_callback(self, request, response):
        self.get_logger().info("Running sleep script...")
        try:
            self._stop_ongoing_processes()
            sleep_script = os.path.join(self.scripts_dir, "sleep.py")
            if not os.path.exists(sleep_script):
                raise FileNotFoundError(f"{sleep_script} not found.")
            self.sleep_process = subprocess.Popen(["python3", sleep_script])
            response.success = True
            response.message = "Sleep program started."
        except Exception as e:
            response.success = False
            response.message = f"Error running sleep script: {str(e)}"
        return response

    def _run_auto_record_callback(self, request, response):
        task_name = request.task_name
        num_episodes = request.num_episodes
        self.get_logger().info(f"Auto record: {task_name} ({num_episodes} episodes)")

        if task_name not in self.valid_tasks:
            response.success = False
            response.message = f"Task '{task_name}' is not valid."
            return response

        try:
            script_path = os.path.join(self.scripts_dir, "auto_record.sh")
            if not os.path.exists(script_path):
                raise FileNotFoundError(f"{script_path} not found.")
            command = f"bash {script_path} {task_name} {num_episodes}"
            self.auto_record_process = subprocess.Popen(command, shell=True)
            response.success = True
            response.message = "Auto record started."
        except Exception as e:
            response.success = False
            response.message = f"Error starting auto record: {str(e)}"
        return response

    def _stop_ongoing_processes(self):
        for proc in [self.auto_record_process, self.sleep_process]:
            if proc:
                proc.terminate()
                proc.wait()
        self.auto_record_process = None
        self.sleep_process = None
        self.get_logger().info("Stopped active subprocesses.")

    def destroy_node(self):
        self._stop_ongoing_processes()
        super().destroy_node()


def main():
    rclpy.init()
    node = NodeManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
