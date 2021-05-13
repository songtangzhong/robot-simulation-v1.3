# robot-simulation-v1.3
The project is based on ros2-foxy and gazebo-11. This is the basic version of the project, which contains all basic interfaces and operating functions to operate a robot.

## Usage:
1. Clone the project.
```
cd ~
mkdir -p robot-simulation-v1.3/src
cd robot-simulation-v1.3/src
git clone https://github.com/songtangzhong/robot-simulation-v1.3.git
cd robot-simulation-v1.3
mv ./* ../
cd ..
rm -rf robot-simulation-v1.3
```

2. Create following files in you /usr/local folder.
```
sudo mkdir /usr/local/robot_files
sudo touch /usr/local/robot_files/robot_arm_shm
sudo touch /usr/local/robot_files/robot_arm_sem
sudo touch /usr/local/robot_files/robot_end_effector_shm
sudo touch /usr/local/robot_files/robot_end_effector_sem
sudo touch /usr/local/robot_files/robot_state_shm
sudo touch /usr/local/robot_files/robot_state_sem
```

3. Build the workspace.
```
cd ~/robot-simulation-v1.3
colcon build --ament-cmake-args -DEND_EFFECTOR=true
```
    If you don't use end-effector, run the following command.
```
colcon build --ament-cmake-args -DEND_EFFECTOR=false
```

4. Add following commands to you ~/.bashrc.
```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/robot-simulation-v1.3/src/gazebo_sim_env
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/robot-simulation-v1.3/install/gazebo_sim_env/lib
source ~/.bashrc
```

5. Run the project (in different terminals), don't forget to source you ros2 environment firstly.
```
cd ~/robot-simulation-v1.3
source install/setup.bash
gazebo src/gazebo_sim_env/world/panda.world
ros2 launch robot_hw_interface robot_hw_interface.launch.py
```

6. Test if all the controllers have been loaded, started and configured successfully.
```
ros2 control list_hardware_interfaces
ros2 control list_controllers
```

7. Test robot state callback function.
```
ros2 run robot_fun test_trigger
```

8. Test some interface functions.
```
ros2 run robot_fun test_set_arm_joint_positions
```

9. Test switch controllers.
```
ros2 run robot_hw_interface test_switch_controller effort_controllers position_controllers
ros2 run robot_hw_interface test_swtich_controller velocity_controllers effort_controllers
ros2 run robot_hw_interface test_swtich_controller position_controllers velocity_controllers 
```

10. Detailed illustration of the project will be added in the future.