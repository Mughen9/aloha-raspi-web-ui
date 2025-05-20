# ALOHA Control System

The ALOHA Control System is a modular, ROS 2-based software suite designed for the orchestration, visualization, and recording of robotic tasks. It includes both hardware-local and web-based frontends, supported by a centralized Node Manager backend service that handles bringup, execution control, and camera feeds.

---

## Repository Structure

```
aloha-raspi-web-ui/
├── backend/              # ROS 2 Node Manager (Python)
├── frontend/
│   ├── ALOHA_GUI_TOUCH.py  # Touchscreen interface for Raspberry Pi
│   ├── Web_page.html       # Browser-based web frontend
├── docs/
│   └── images/           # UML diagrams and flowcharts
├── README.md             # Project overview (this file)
```

---

## System Overview

This system consists of:

- **Node Manager (Backend):**  
  A ROS 2 Python node that exposes services for launching the robot bringup stack, running a sleep mode, and initiating automated recording of tasks.

- **Frontend Interfaces:**  
  - **Raspberry Pi GUI:** Built using `pygame`, optimized for local touch-based operation.
  - **Web Interface:** HTML + JavaScript UI that communicates over WebSockets via `rosbridge_server`.

---

## Functional Architecture

The following diagram provides a high-level view of how the frontend interfaces interact with the backend:

![System Flow]\backend\Images/backend-flow.png)

---

## Frontend Architecture

### Raspberry Pi Interface

- Optimized for 1024x600 touch displays
- Built using `pygame` and `rclpy`
- Communicates directly with Node Manager services

![Raspberry Pi Flow](docs/images/raspi-flow.png)

### Web Interface

- HTML5 interface styled with TailwindCSS
- Communicates with backend via `rosbridge_server`
- Real-time canvas views for multiple camera feeds

![Web Class Diagram](docs/images/html-class.png)  
![Camera Feed Flow](docs/images/html-camera.png)

---

## Node Manager Services

The backend exposes the following ROS 2 services:

| Service Name         | Type        | Description                            |
|----------------------|-------------|----------------------------------------|
| `launch_ros2`        | Trigger     | Launches full robot bringup stack      |
| `run_sleep`          | Trigger     | Executes local sleep script            |
| `run_auto_record`    | AutoRecord  | Starts task recording with arguments   |

The service logic is implemented via Python `subprocess` calls and coordinated to avoid conflicting execution.

---

## Development Setup

Ensure you have:

- ROS 2 (Foxy or Humble)
- `rosbridge_server` installed and available
- For the web interface: serve `Web_page.html` in a browser
- For the Pi interface: run `ALOHA_GUI_TOUCH.py` on a display-equipped Pi

Example launch command:

```bash
ros2 run backend node_manager
```

To open the web UI:

```bash
firefox frontend/Web_page.html
```

---

## Documentation & Diagrams

Architecture and behavior diagrams are available under:

```
docs/images/
├── html-class.png
├── html-camera.png
├── html-uml.png
├── raspi-flow.png
├── backend-flow.png
```

Use these for internal references, presentations, or technical onboarding.

---

## Maintainer

Sai  


For contributions or inquiries, please open a GitHub issue or contact directly.
