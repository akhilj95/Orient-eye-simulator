clear variables
close all

% Run this to set all the required simulation parameters

%struct flags
flags.head = 0; % Set 1 for free moving head ($ Currently not implemented $)
flags.wrapping = 0;  % Set 1 to include cable wrapping
flags.visual = 0;  % Set 1 for rviz visualization ($ Doesn't have cable wrapping $)
flags.plot = 1;  % Set 1 for plots. The exact plots can be changed in physics.m

%struct initial_state
initial_state.R_eye = eye(3); % Initial orientation of eye in world frame
initial_state.omega_eye = zeros(3,1); % Initial eye angular velocity(x y z)
initial_state.R_head = eye(3); % Initial orientation of head in world frame
initial_state.omega_head = zeros(3,1); % Initial head angular velocity(x y z)

Ts = 0.001; % simulation timestep of 1 ms

theta = [2.01893712481317,1.96299387485996,1.98106292418320,2.03700617391867,1.97146423659847,2.02853581254045];
motor_commands = repmat(theta,1000,1);

[final_state,history_state, history_tau_eye]=physics(flags, initial_state,  Ts, motor_commands);