# Executor Worker Package

## Description
The **`executor_worker`** package serves as the physical execution layer for the **Justina** robot. Its primary responsibility is to receive high-level goals (actions) from the `Executor Manager`, process them using specialized toolsets from `executor_tools`, and report the final execution status back to the supervision system.

This node implements a **ROS 2 Action Server** that enables asynchronous execution of complex tasks, ensuring the robot can perform sequential actions within a generated plan.

---

## Key Features
* **Dynamic Tool Mapping**: Utilizes an `actionMap` based on C++ lambdas to register tools dynamically (`Listen`, `Say`, `GoTo`, `Follow`, etc.), making the system highly extensible.
* **Robust Error Handling**: Implements an **"Accept-to-Report"** policy. Even invalid actions are accepted by the ROS 2 protocol to ensure a formal report (`authorize reject`) is sent to the Supervisor before the task is aborted.
* **Multi-threaded Execution**: Each accepted task runs in a separate detached thread (`std::thread`), preventing the node's main executor from blocking during long-running tasks.

---

## Supported Actions
The node maps command strings to specialized classes within `executor_tools`:

| Command | Tool Class | Description |
| :--- | :--- | :--- |
| `listen` | `Listen` | Audio capture and Natural Language Processing (NLP). |
| `say` | `Say` | Text-to-Speech (TTS) for Human-Robot Interaction. |
| `goto` | `GoTo` | Autonomous navigation to known locations. |
| `follow` | `Follow` | Activation of the human tracking service. |
| `check` | `Check` | Hardware status verification (e.g., door sensors). |
| `save` | `Save` | Storing information into the Knowledge Bank. |

---

## Execution Architecture

1. **Goal Handling**: The node accepts all goals to guarantee that the reporting system (`Executor::authorize`) always has an opportunity to execute.
2. **Dispatching**: The system looks up the requested action in the `actionMap`.
3. **Authorization & Execution**: 
    * **Valid Action**: The specific tool executes its logic and reports success/failure.
    * **Invalid Action**: The node creates a temporary `Executor` instance to send a rejection message to the Supervisor and subsequently aborts the task.

---

## Dependencies
This package depends on the following tools and interfaces from the Justina repository:
* `rclcpp` & `rclcpp_action`
* `executor_interfaces` (Action definitions)
* `executor_tools` (Tool logic implementation)
* `hri_tools`, `navigation_tools`, `hardware_tools`

---

## Usage

To launch the node individually:
```bash
ros2 run executor_worker executor_worker_node