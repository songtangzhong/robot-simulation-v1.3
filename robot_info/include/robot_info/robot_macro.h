#ifndef ROBOT_MACRO_H_
#define ROBOT_MACRO_H_

#define ARM_DOF 7
#define ARM_SHM_FILE "/usr/local/robot_files/robot_arm_shm" 
#define ARM_SEM_FILE "/usr/local/robot_files/robot_arm_sem"

#ifdef USE_END_EFFECTOR
#define END_EFF_DOF 2
#define END_EFF_SHM_FILE "/usr/local/robot_files/robot_end_effector_shm"
#define END_EFF_SEM_FILE "/usr/local/robot_files/robot_end_effector_sem"
#endif

#define ROBOT_STATE_SHM_FILE "/usr/local/robot_files/robot_state_shm"
#define ROBOT_STATE_SEM_FILE "/usr/local/robot_files/robot_state_sem"

#endif