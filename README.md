# robot-simulation-v1.3

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

2. Create following files in you /dev folder.
```
sudo touch /dev/robot_arm_shm
sudo touch /dev/robot_arm_sem
sudo touch /dev/robot_end_effector_shm
sudo touch /dev/robot_end_effector_sem
sudo touch /dev/robot_state_shm
sudo touch /dev/robot_state_sem
```

3. Add following commands to you ~/.bashrc.
```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/robot-simulation-v1.3/src/gazebo_sim_env
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/robot-simulation-v1.3/install/gazebo_sim_env/lib
source ~/.bashrc
```

4. Build the workspace.
```
cd ~/robot-simulation-v1.3
colcon build --ament-cmake-args -DEND_EFFECTOR=true
```

5. Run the project (in different terminals), donn't forget to source you ros2 environment firstly.
```
gazebo ~/robot-simulation-v1.3/src/gazebo_sim_env/world/panda.world
ros2 launch robot_hw_interface robot_hw_interface.launch.py
```

6. Test robot interface function.
```
ros2 run robot_info test_trigger
```

7. Test position control mode.
```
ros2 topic pub /position_controllers/commands std_msgs/msg/Float64MultiArray "data:                                    
- 0
- 0
- 0
- 0
- 0
- 0
- 0"
```

8. Test switch controller.
```
ros2 run robot_hw_interface test_swtich_controller effort_controllers position_controllers
ros2 run robot_hw_interface test_swtich_controller velocity_controllers effort_controllers
ros2 run robot_hw_interface test_swtich_controller position_controllers velocity_controllers 
```

9. More APIs to directly operate robot are in package "robot_fun".

10. Detailed illustration will be added in the future.