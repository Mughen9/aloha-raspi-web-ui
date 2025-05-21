
# ALOHA Raspberry Pi Touch Interface

A fullscreen, local touchscreen interface for Raspberry Pi, enabling real-time interaction with the ALOHA robot via ROS 2 service calls.

---

## System Overview

```mermaid
graph TD
User[User Touch Input] -->|Trigger| GUI[ALOHA GUI]
GUI -->|call_async| ROSClient
ROSClient -->|/launch_ros2| Backend
ROSClient -->|/run_sleep| Backend
ROSClient -->|/run_auto_record| Backend
```

---

## Component Diagram

```mermaid
classDiagram
class TouchGUI {
  - screen
  - buttons
  - input_boxes
  - task_input
  - episode_input
  - status
  + run()
  + handle_touch_or_click()
  + handle_button()
}

class ROSClient {
  + call_trigger(client)
  + call_auto_record(task_name, num_episodes)
}

TouchGUI --> ROSClient : uses
```

---

## File Location

```
frontend/
└── ALOHA_GUI_TOUCH.py
```

---



## Execution

```bash
python3 ALOHA_GUI_TOUCH.py
```

Preconditions:
- ROS 2 sourced
- Backend node_manager running

---

## Requirements

- pygame
- ROS 2 (rclpy, std_srvs, aloha_msgs)

---

## Author

Sai 
