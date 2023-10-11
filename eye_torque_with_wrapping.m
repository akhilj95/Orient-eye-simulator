function [tau_eye, delta_l, cable_forces, slack_flags, DeadZone]= eye_torque_with_wrapping(R_eye_in_head, omega_eye,omega_head,thetas,eye,muscle)

DeadZone = 0; %Force velocity to zero, if desired

% Transforming the head insertion points to eye reference frame
H_eyeframe = R_eye_in_head'*muscle.H;

% Tangent points of cable on eye
T=get_tangent_points(H_eyeframe,muscle.E,eye.R);
% Angle of arc between eye insertion and tangent points
[~,alpha] = get_arc_angle(T,muscle.E,10); 

vector_E_to_H = H_eyeframe-muscle.E;
l_E_to_H = vecnorm(vector_E_to_H);
vector_T_to_H = H_eyeframe-T;
l_T_to_H = vecnorm(vector_T_to_H);

% Checking and replacing required values if l_E_to_H>l_T_to_H
pos_change=l_E_to_H>l_T_to_H;
vector_E_to_H(:,pos_change) = vector_T_to_H(:,pos_change);
l_E_to_H(pos_change)=l_T_to_H(pos_change);
l_E_to_H = l_E_to_H + (eye.R*alpha).*double(pos_change);

% current string lengths
l = l_E_to_H + muscle.d + muscle.r*thetas;

%check for slack
delta_l = l - muscle.l0;
slack_flags=int8(delta_l<0); % checking slack condition

v = vector_E_to_H./vecnorm(vector_E_to_H); % unit force direction

F = ((muscle.k*max(0,delta_l))./muscle.l0).*v; % cable forces with directions

cable_forces=vecnorm(F); % cable force magnitudes

tau=cross(muscle.E,F); 

tau_muscles = sum(tau,2); % total elastic torque

tau_damping = -eye.D*(omega_eye - R_eye_in_head'*omega_head); %damping torque

tau_eye = tau_muscles + tau_damping; % total eye torque

%stop condition
static_friction_torque = 0.005;
if norm(omega_eye)< 0.01
    if norm(tau_eye) < static_friction_torque
        tau_eye = zeros(3,1); 
        %instead put omega_eye = 0
        %         DeadZone = 1;
    end
end