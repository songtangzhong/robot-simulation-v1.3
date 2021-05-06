#ifndef ROBOT_MACRO_H_
#define ROBOT_MACRO_H_

#define ARM_DOF 7
#define ARM_SHM_KEY 1234
#define ARM_SEM_KEY 1235

#define END_EFF_DOF 2
#define END_EFF_SHM_KEY 2234
#define END_EFF_SEM_KEY 2235

#define ROBOT_STATE_SHM_KEY 3234
#define ROBOT_STATE_SEM_KEY 3235

// Macro to control if the end-effector is used,
// which has no influence on this package.
#define USE_END_EFFECTOR

#endif