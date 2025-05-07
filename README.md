# Crazyflie Firmware (A2R)

Custom crazyflie firmware to work with crazyswarm2

### Installation (crazyswarm2)
https://imrclab.github.io/crazyswarm2/installation.html

### Running program (tip: use 2 terminal tabs)

#### Terminal 1 (ROS):

1. Activate virtual environment:
```bash
source /home/brandon-pae/venv/bin/activate
```

2. Build and run ROS:
```bash
cd ~/ros2_ws

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

. install/local_setup.bash

ros2 launch crazyflie_examples launch.py script:=hello_world
```

To remove cache (delete temp files):
```bash
rm -rf build install log
```

#### Terminal 2 (crazyflie firmware):

1. Activate virtual environment and navigate to firmware directory:
```bash
source /home/brandon-pae/venv/bin/activate

cd ~/Desktop/brandon/a2r-crazyflie-firmware
```

2. Build and flash firmware:
```bash
make defconfig

cd examples/app_tinympc_controller

make clean

make -j$(nproc)  # For Linux
# OR
make -j$(sysctl -n hw.ncpu)  # For Mac

make cload
```

### File Structure:

- `app_tinympc_controller/`
  - Main folder for the tinympc controller
  - `src/`
    - `controller_tinympc.cpp`
- `TinyMPC/`
  - `src/tinympc/`
    - Contains the admm and kbuild file (MUST UPDATE**)

### TinyMPC Submodule Setup

TinyMPC is added as a submodule and must be modified following these instructions:

1. Kbuild file:
```makefile
obj-y += admm.o
obj-y += rho_benchmark.o
```

2. May need to comment out parts of the `admm.cpp` file to get it to compile

### Next Steps

1. Compare the existing controller implementation:
   - Current file: `controller_tinympc.cpp`
   - Reference file: [controller_tinympc.cpp from crazyswarm1](https://github.com/ishaanamahajan/tinympc-crazyflie-firmware/blob/main/examples/controller_tinympc_eigen_task/src/controller_tinympc.cpp)

2. Focus on updating the controller initialization:
   - Target method: `void controllerOutOfTreeInit(void)`
   - Gradually add back functionality
   - Note: Some structs and methods may need to be updated for compatibility