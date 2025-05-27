import os
import json
import pygame
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from aloha_msgs.srv import AutoRecord

class ROSClient(Node):
    def __init__(self):
        super().__init__('aloha_touch_gui_client')
        self.cli_launch = self.create_client(Trigger, 'launch_ros2')
        self.cli_sleep = self.create_client(Trigger, 'run_sleep')
        self.cli_record = self.create_client(AutoRecord, 'run_auto_record')

        for client, name in [
            (self.cli_launch, 'launch_ros2'),
            (self.cli_sleep, 'run_sleep'),
            (self.cli_record, 'run_auto_record')
        ]:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {name} service...')

    def call_trigger(self, client):
        request = Trigger.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def call_auto_record(self, task_name, num_episodes):
        request = AutoRecord.Request()
        request.task_name = task_name
        request.num_episodes = num_episodes
        future = self.cli_record.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

class TouchGUI:
    def __init__(self):
        pygame.init()
        pygame.mouse.set_visible(True)
        self.screen = pygame.display.set_mode((1024, 600), pygame.FULLSCREEN | pygame.NOFRAME)
        pygame.display.set_caption("ALOHA Node Manager")
        self.font = pygame.font.Font(None, 36)
        self.small_font = pygame.font.Font(None, 28)

        self.buttons = {
            'launch': pygame.Rect(40, 40, 300, 60),
            'sleep': pygame.Rect(40, 120, 300, 60),
            'record': pygame.Rect(40, 200, 300, 60),
            'quit': pygame.Rect(40, 420, 300, 60),
        }
        self.input_boxes = {
            'task': pygame.Rect(40, 280, 300, 50),
            'episodes': pygame.Rect(40, 340, 180, 50)
        }
        self.inc_button = pygame.Rect(230, 340, 50, 50)
        self.dec_button = pygame.Rect(290, 340, 50, 50)

        self.task_input = ''
        self.episode_input = '1'
        self.active_input = None
        self.status = "Waiting for input..."
        self.sleep_done = False

        self.task_options = self.load_task_options()
        self.scroll_offset = 0
        self.max_visible_options = 4

        self.ros_client = None
        self.executor = None

        self.cameras = {
            'Left Cam': pygame.Rect(400, 40, 300, 160),
            'Right Cam': pygame.Rect(720, 40, 280, 160),
            'Up Cam': pygame.Rect(400, 220, 300, 160),
            'Low Cam': pygame.Rect(720, 220, 280, 160),
        }

    def load_task_options(self):
        task_config_path = os.getenv("ALOHA_TASK_CONFIG", "./task_config.json")
        try:
            with open(task_config_path, "r") as f:
                config = json.load(f)
                return config.get("valid_tasks", [])
        except Exception as e:
            print(f"Failed to load task options from {task_config_path}: {e}")
            return []

    def run(self):
        def ros_spin_thread():
            rclpy.init()
            self.ros_client = ROSClient()
            self.executor = MultiThreadedExecutor()
            self.executor.add_node(self.ros_client)
            self.executor.spin()

        threading.Thread(target=ros_spin_thread, daemon=True).start()

        clock = pygame.time.Clock()
        running = True

        while running:
            self.screen.fill((15, 15, 25))
            click_pos = None
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    click_pos = event.pos
                    self.handle_touch_or_click(event.pos)
                elif event.type == pygame.FINGERDOWN:
                    x = int(event.x * self.screen.get_width())
                    y = int(event.y * self.screen.get_height())
                    click_pos = (x, y)
                    self.handle_touch_or_click((x, y))

            for name, rect in self.buttons.items():
                pygame.draw.rect(self.screen, (70, 130, 180), rect)
                label = self.font.render(name.upper(), True, (255, 255, 255))
                self.screen.blit(label, (rect.x + 20, rect.y + 15))

            for name, rect in self.input_boxes.items():
                pygame.draw.rect(self.screen, (40, 40, 40), rect)
                pygame.draw.rect(self.screen, (255, 255, 255), rect, 2)
                text = self.task_input if name == 'task' else self.episode_input
                label = self.small_font.render(text, True, (255, 255, 255))
                self.screen.blit(label, (rect.x + 10, rect.y + 10))

            pygame.draw.rect(self.screen, (100, 180, 100), self.inc_button)
            pygame.draw.rect(self.screen, (100, 180, 100), self.dec_button)
            self.screen.blit(self.small_font.render('+', True, (0, 0, 0)), (self.inc_button.x + 18, self.inc_button.y + 10))
            self.screen.blit(self.small_font.render('-', True, (0, 0, 0)), (self.dec_button.x + 18, self.dec_button.y + 10))

            if self.active_input == 'task':
                dropdown_start_y = 420
                for i in range(self.scroll_offset, min(len(self.task_options), self.scroll_offset + self.max_visible_options)):
                    option_rect = pygame.Rect(360, dropdown_start_y + (i - self.scroll_offset) * 45, 300, 40)
                    pygame.draw.rect(self.screen, (60, 60, 60), option_rect)
                    pygame.draw.rect(self.screen, (255, 255, 255), option_rect, 1)
                    label = self.small_font.render(self.task_options[i], True, (255, 255, 255))
                    self.screen.blit(label, (option_rect.x + 10, option_rect.y + 10))
                    if click_pos and option_rect.collidepoint(click_pos):
                        self.task_input = self.task_options[i]
                        self.active_input = None
                        break

            for name, rect in self.cameras.items():
                pygame.draw.rect(self.screen, (10, 10, 10), rect)
                pygame.draw.rect(self.screen, (255, 255, 255), rect, 1)
                label = self.small_font.render(name, True, (255, 255, 255))
                self.screen.blit(label, (rect.x + 10, rect.y + 10))

            status_label = self.small_font.render(self.status, True, (0, 255, 0))
            self.screen.blit(status_label, (40, 540))

            pygame.display.flip()
            clock.tick(30)

        pygame.quit()
        rclpy.shutdown()

    def handle_touch_or_click(self, pos):
        if self.inc_button.collidepoint(pos):
            try:
                val = int(self.episode_input) if self.episode_input.isdigit() else 0
                self.episode_input = str(val + 1)
            except:
                self.episode_input = '1'
        elif self.dec_button.collidepoint(pos):
            try:
                val = int(self.episode_input) if self.episode_input.isdigit() else 1
                self.episode_input = str(max(val - 1, 1))
            except:
                self.episode_input = '1'

        for name, rect in self.buttons.items():
            if rect.collidepoint(pos):
                if name == 'quit':
                    if self.sleep_done:
                        pygame.quit()
                        rclpy.shutdown()
                        exit(0)
                    else:
                        self.status = "Sleep must be triggered first."
                else:
                    self.handle_button(name)
        for name, rect in self.input_boxes.items():
            if rect.collidepoint(pos):
                self.active_input = name

    def handle_button(self, name):
        if not self.ros_client:
            self.status = "ROS client not ready."
            return
        if name == 'launch':
            result = self.ros_client.call_trigger(self.ros_client.cli_launch)
            self.status = result.message
        elif name == 'sleep':
            result = self.ros_client.call_trigger(self.ros_client.cli_sleep)
            self.status = result.message
            self.sleep_done = True
        elif name == 'record':
            if not self.task_input or not self.episode_input.isdigit():
                self.status = "Invalid input."
                return
            result = self.ros_client.call_auto_record(self.task_input, int(self.episode_input))
            self.status = result.message


if __name__ == "__main__":
    gui = TouchGUI()
    gui.run()
