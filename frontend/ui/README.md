
# ALOHA Web Interface

A responsive, browser-based UI that interacts with the ALOHA robot system over WebSocket using ROSLIB.js.

---

## System Overview

```mermaid
graph TD
BrowserUser -->|Web UI| WebPage
WebPage -->|WS 9090| RosbridgeServer
RosbridgeServer -->|Service Call| ROSNode
```

---

## Sequence Flow

```mermaid
sequenceDiagram
participant User
participant WebUI
participant Rosbridge
participant NodeManager

User->>WebUI: Click Button
WebUI->>Rosbridge: Call /launch_ros2
Rosbridge->>NodeManager: Trigger Request
NodeManager-->>Rosbridge: Response
Rosbridge-->>WebUI: Status Update
WebUI-->>User: Display Result
```

---

## Component Diagram

```mermaid
classDiagram
class HTML_UI {
  + buttons
  + inputs
  + WebSocket status
  + canvas streams
}

class RosLib {
  + connect(ws://localhost:9090)
  + callService()
  + subscribe(topic)
}

HTML_UI --> RosLib : uses
```

---

## File Location

```
frontend/
└── Web_page.html
```

---

## Requirements

- rosbridge_server
- ROS 2 topics & services
- JavaScript (RosLib.js)
- TailwindCSS (CDN)

---

## Execution

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
firefox Web_page.html
```

---

## Author

 Sai 
