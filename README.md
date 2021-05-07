# robot-simulation-v1.3

## Usage:
1. clone the project.
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

4. build the workspace
```
cd ~/robot-simulation-v1.3
colcon build
```