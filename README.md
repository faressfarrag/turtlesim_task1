# 🐢 My Turtle Autonomy (ROS2)

This package contains a ROS2 node that makes the turtle in the
`turtlesim` simulator move **autonomously without hitting the walls**.\
Additionally, pressing the **Space bar** will return the turtle to the
origin `(0,0)`.

------------------------------------------------------------------------

## 📦 Package Structure

    ros2_ws/src/my_turtle_pkg
    ├── launch
    │   └── turtle_autonomy_launch.py
    ├── my_turtle_pkg
    │   ├── __init__.py
    │   └── turtle_autonomy.py
    ├── package.xml
    ├── resource
    │   └── my_turtle_pkg
    ├── setup.cfg
    ├── setup.py
    └── test

------------------------------------------------------------------------

## 🚀 Setup Steps

### 1. Create Workspace

``` bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### 2. Create Package

``` bash
ros2 pkg create --build-type ament_python my_turtle_pkg
```

### 3. Add Files

-   Add `turtle_autonomy.py` inside `my_turtle_pkg/`
-   Add `turtle_autonomy_launch.py` inside `launch/`
-   Update `setup.py` and `package.xml` correctly

### 4. Build Package

``` bash
cd ~/ros2_ws
colcon build --packages-select my_turtle_pkg
source install/setup.bash
```

### 5. Run

Open **two terminals**:

**Terminal 1 -- Start turtlesim**

``` bash
ros2 run turtlesim turtlesim_node
```

**Terminal 2 -- Run autonomy node**

``` bash
ros2 run my_turtle_pkg turtle_autonomy
```

------------------------------------------------------------------------

## 🎮 Controls

-   By default, the turtle **wanders around** without hitting walls.\
-   Press **Space bar** → the turtle drives back to **(0,0)**, then
    resumes wandering.

------------------------------------------------------------------------

## ⚠️ Problems Faced & Fixes

1.  **Error: `file 'turtle_autonomy_launch.py' not found`**
    -   Cause: Wrong package structure.\
    -   Fix: Ensure `launch/` folder is inside the package, rebuild with
        `colcon build`.
2.  **Build failed -- `NameError: name 'os' is not defined` in
    setup.py**
    -   Cause: Missing `import os` in `setup.py`.\
    -   Fix: Added `import os`.
3.  **`keyboard` module not working**
    -   Cause: `keyboard` package requires root permissions.\
    -   Fix: Replaced with `termios + select` to read keyboard input in
        terminal.
4.  **Returning to wrong location**
    -   Initially returned to `(5.5, 5.5)` (center).\
    -   Changed to `(0,0)` for the bonus task.

------------------------------------------------------------------------

## ✅ Features

-   Autonomous wandering with wall avoidance.\
-   Space bar to return to origin `(0,0)`.\
-   Resumes wandering after reaching origin.

------------------------------------------------------------------------

## 📌 Next Ideas

-   Add a node that publishes keyboard inputs as ROS2 messages (works
    with `ros2 launch`).\
-   Make turtle draw a circle when it reaches origin.\
-   Multi-turtle version for cooperative behavior.
