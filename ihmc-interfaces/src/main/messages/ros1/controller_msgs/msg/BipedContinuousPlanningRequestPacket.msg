# This message is part of the IHMC quadruped footstep planning module.

int8 FOOTSTEP_PLANNER_TARGET_TYPE_POSE_BETWEEN_FEET=0

int8 FOOTSTEP_PLANNER_TARGET_TYPE_FOOTSTEPS=1

int32 UNSET_ID=-1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

int8 start_target_type

geometry_msgs/Point left_start_position_in_world

geometry_msgs/Quaternion left_start_orientation_in_world

geometry_msgs/Point right_start_position_in_world

geometry_msgs/Quaternion right_start_orientation_in_world

geometry_msgs/Point goal_position_in_world

geometry_msgs/Quaternion goal_orientation_in_world

# Field default value -1
int32 planner_request_id

float64 timeout

float64 best_effort_timeout

# Field default value -1
int32 max_iterations

float64 horizon_length


