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

## 🚀 Full Step-by-Step Instructions

### 1. Create a ROS2 Workspace

``` bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### 2. Create the Package

``` bash
ros2 pkg create --build-type ament_python my_turtle_pkg
```

This will generate the basic package structure.

------------------------------------------------------------------------

### 3. Add Source Code

-   Inside `my_turtle_pkg/my_turtle_pkg/`, create a file named
    `turtle_autonomy.py` with the node code.
-   Inside `my_turtle_pkg/launch/`, create `turtle_autonomy_launch.py`
    to launch the node.

------------------------------------------------------------------------

### 4. Update `setup.py`

Make sure `setup.py` includes:

``` python
import os
from setuptools import setup

package_name = 'my_turtle_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ['launch/turtle_autonomy_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Autonomous turtle in turtlesim',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'turtle_autonomy = my_turtle_pkg.turtle_autonomy:main',
        ],
    },
)
```

> **Note**: Initially, we forgot `import os`, which caused a build
> error.

------------------------------------------------------------------------

### 5. Build the Package

``` bash
cd ~/ros2_ws
colcon build --packages-select my_turtle_pkg
source install/setup.bash
```

------------------------------------------------------------------------

### 6. Run the Nodes

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

-   By default, the turtle **wanders around** avoiding walls.\
-   Press **Space bar** → the turtle drives back to **(0,0)**.\
-   After reaching origin → it resumes wandering.

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
