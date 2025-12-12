# Header
std_msgs/Header header
# MPC finite state machine state
std_msgs/Int64 fsm_state_id
std_msgs/String fsm_state
# Solving time consuming
float64 mpc_solve_time
float64 suggest_hover_percentage

mars_quadrotor_msgs/QuadrotorState reference
mars_quadrotor_msgs/QuadrotorState command
mars_quadrotor_msgs/QuadrotorState feedback
mars_quadrotor_msgs/QuadrotorState error


