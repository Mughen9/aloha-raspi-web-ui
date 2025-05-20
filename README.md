# ALOHA Control System

The ALOHA Control System is a modular, ROS 2-based framework designed for robotic task orchestration. It features a central backend service (Node Manager) and two user interfaces: a touchscreen GUI for Raspberry Pi and a browser-based HTML interface. These components are organized for clarity and scalability in real-world deployments.

---

## Repository Structure

```
aloha-raspi-web-ui/
├── backend/
│   ├── node_manager.py
│   └── Images/
│       ├── backend flow.png
│       └── Overall_Work_Flow_Architecture.png
├── frontend/
│   ├── raspi/
│   │   ├── ALOHA_GUI_TOUCH.py
│   │   └── Images/
│   │       ├── raspi flow.png
│   │       ├── FLOW.png
│   │       ├── ROS_GUI_TOUCH_UML.png
│   │       └── ALOHA_GUI_TOUCH_PANEL_VIEW.jpeg
│   ├── ui/
│   │   ├── Web_page (1).html
│   │   └── Images/
│   │       ├── html class.png
│   │       ├── html uml.png
│   │       ├── html camera.png
│   │       └── html system contetx.png
```

---

## System Overview

This system includes:

- **Backend Node Manager**: A ROS 2 node providing services to control robot state, record tasks, and handle system execution.
- **Frontend Interfaces**:
  - **Raspberry Pi GUI**: Touchscreen-based local interface built with Python and pygame.
  - **Web UI**: HTML and JavaScript-based interface for browser interaction using WebSockets.

---

## Backend Architecture

The backend handles ROS 2 services for launching, sleeping, and task recording. Subprocesses are managed to ensure controlled execution.

**Architecture Diagram**  
![Backend Flow](backend/Images/backend%20flow.png)

---

## Raspberry Pi GUI

This interface supports touch input and communicates directly with the backend using ROS 2 Python clients.

**Flow Diagram**  
![Raspberry Pi Flow](frontend/raspi/Images/raspi%20flow.png)

---

## Web Interface

The HTML UI is accessible through any modern browser and communicates with the ROS backend using `rosbridge_server`. Camera feeds and control buttons are integrated for seamless operation.

**Class Diagram**  
![Web Class Diagram](frontend/ui/Images/html%20class.png)

**Camera Feed Flow**  
![Camera Flow](frontend/ui/Images/html%20camera.png)

**System Context**  
![Web System Context](frontend/ui/Images/html%20system%20contetx.png)

---

## Launch Instructions

Ensure the ROS 2 environment is sourced before executing:

**Backend Node Manager**
```bash
ros2 run backend node_manager
```

**Raspberry Pi Interface**
```bash
python3 frontend/raspi/ALOHA_GUI_TOUCH.py
```

**Web UI**
```bash
firefox frontend/ui/Web_page\ \(1\).html
```

Make sure `rosbridge_websocket` is running for the web interface to communicate properly.

---

## Documentation

All supporting UML diagrams and architecture visuals are located within the respective `Images/` directories of each module.
