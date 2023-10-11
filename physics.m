function [final_state,history_state, history_tau_eye]= physics(flags, initial_state,  Ts, motor_commands)

% % % Things not done
% visualization of cable wrapping
% head torque
% arc angle code optimization
% Deadzone

% Initialization
eye_torque = [0;0;0];
head_torque = [0;0;0];
omega_head = initial_state.omega_head;
omega_eye = initial_state.omega_eye;
R_eye_in_world=initial_state.R_eye;
R_head_in_world=initial_state.R_head;

simulation_stepsize = Ts;
p = size(motor_commands,1); % total simulation steps

% Physical properties of eye and head
eye.inertia = 1*[ 0.0004759 0 0; 0 0.0004316 0;0 0 0.0003956];
eye.D = 2*[0.02 0 0; 0 0.02 0; 0 0 0.02]; %Damping matrix for the eye
eye.R = 0.079/2; % radius of eye
head.inertia = [0.181 0 0; 0  0.215 0; 0 0 0.142];
head.D = 3*[0.55 0 0; 0 0.55 0; 0 0 0.55]; %Damping matrix for head

% Rotation matrix to rotate insertion points to the right coordinate frame
a = [1 0 0; 0 cos(-90*pi/180) -sin(-90*pi/180); 0 sin(-90*pi/180) cos(-90*pi/180)];

% Insertion points on the eye
Q1 = a*[-0.002; 0.040; -0.0014]; %IR in m
Q2 = a*[0.0077; 0; 0.0393]; %MR in m
Q3 = a*[-0.002; -0.040; 0.0008]; %SR in m
Q4 = a*[0.0077; 0; -0.0393]; %LR in m
Q5 = a*[-0.0112; 0.0383; -0.0014]; %IO in m
Q6 = a*[-0.0118; -0.0381; -0.0012]; %SO in m

% Muscle initial point (at head)
S1 = a*[-0.1001; 0.0078; 0.0407]; %IR in m
S2 = a*[-0.1001; -0.0035; 0.0516]; %MR in m
S3 = a*[-0.1001; -0.0149; 0.0407]; %SR in m
S4 = a*[-0.1001; -0.0035; 0.0296]; %LR in m
S5 = a*[0.045; 0.062; 0.0375]; %IO in m
S6 = a*[0.045; -0.062; 0.0375]; %SO in m

muscle.H = [S1 S2 S3 S4 S5 S6]; % Muscle starting points from head
muscle.E = [Q1 Q2 Q3 Q4 Q5 Q6]; % Muscle insertion points on eye
muscle.d = [0.04,0.04,0.04,0.04,0.065,0.065]; %distance from the spindle to the motors
muscle.k = 20; % Constant of force
muscle.r = 0.024; % radius of motor spindle

history_state = zeros(p,14);
history_tau_eye=zeros(p,3);
history_tau_head=zeros(p,3);
history_f_eye=zeros(p,6);
history_slack=zeros(p,6);
history_delta_l=zeros(p,6); % only for rviz visualization

%set function for cable force and initial cable length
if flags.wrapping == 0
    eye_torque_handle = @eye_torque;
    muscle.l0 = vecnorm(muscle.H-muscle.E) + muscle.d; % initial string length
else
    eye_torque_handle = @eye_torque_with_wrapping;

    % Calculating l0 for case of cable wrapping on eye
    T = get_tangent_points(muscle.H,muscle.E,eye.R);
    [~,alpha] = get_arc_angle(T,muscle.E,10); % arc angle

    l_E_to_H = vecnorm(muscle.H-muscle.E);
    l_T_to_H = vecnorm(muscle.H-T);
    pos_change=l_E_to_H>l_T_to_H; % muscles for which wrapping happens
    l_E_to_H(pos_change)=l_T_to_H(pos_change);
    l_E_to_H = l_E_to_H + (eye.R*alpha).*double(pos_change);

    muscle.l0 = l_E_to_H + muscle.d; % initial string length
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Start of simulation

if flags.head == 0

    %%%%%% eye only %%%%%%
    for i=1:p % number of iterations

        %computing EYE alpha and omega;
        alpha_eye = (eye.inertia)\(eye_torque - cross(omega_eye,eye.inertia*omega_eye)); %acceleration
        omega_eye = omega_eye+simulation_stepsize*alpha_eye; %angular velocity
        omega_eye_hat = skew(omega_eye);


        %computing rotation matrix for the EYE
        del_R_eye = expm(simulation_stepsize*omega_eye_hat);
        R_eye_in_world = R_eye_in_world*del_R_eye;

        % computing torques for the eye
        [eye_torque, history_delta_l(i,:), history_f_eye(i,:), history_slack(i,:),DeadZone] = eye_torque_handle(R_eye_in_world, omega_eye,omega_head, motor_commands(i,1:6),eye, muscle);

        % Deadzone to be implemented later
        %     if DeadZone
        %         omega_eye = zeros(3,1);
        %     end

        % get quaternion from rotation matrix
        q_eye = rotm2quat(R_eye_in_world);
        q_head = rotm2quat(R_head_in_world);

        %     rot_vec = quat2rod(q_gaze); %orientation in rotation vector

        history_tau_eye(i,:) = eye_torque;

        history_state(i,:) = [q_eye, q_head , omega_eye', omega_head'];

    end

else
    %%%%%% eye & head %%%%%%
    for i=1:p % number of iterations

        %computing EYE alpha and omega;
        alpha_eye = (eye.inertia)\(eye_torque - cross(omega_eye,eye.inertia*omega_eye)); %acceleration
        omega_eye = omega_eye+simulation_stepsize*alpha_eye; %angular velocity
        omega_eye_hat = skew(omega_eye);

        % computing HEAD alpha and omega
        alpha_head = (head.inertia)\(head_torque - cross(omega_head,head.inertia*omega_head)); %acceleration
        omega_head = omega_head+simulation_stepsize*alpha_head; %angular velocity
        omega_head_hat = skew(omega_head);

        %computing rotation matrix for the EYE
        del_R_eye = expm(simulation_stepsize*omega_eye_hat);
        R_eye_in_world = R_eye_in_world*del_R_eye;


        %computing rotation matrix for the HEAD
        del_R_head = expm(simulation_stepsize*omega_head_hat);
        R_head_in_world = R_head_in_world*del_R_head;

        %rotation matrix eye in head
        R_eye_in_head= R_head_in_world'*R_eye_in_world;

        % computing torques for the eye
        [eye_torque, history_delta_l(i,:), history_f_eye(i,:), history_slack(i,:),DeadZone] = eye_torque_handle(R_eye_in_head, omega_eye,omega_head, motor_commands(i,1:6),eye, muscle);

        %     if DeadZone
        %         previous_omega_eye = zeros(3,1);
        %         i
        %     end

        % computing torques for head
        % Currently not implemented
        % head_torque = head_torque(R_eye_in_head, R_head_in_world, omega_eye, omega_head, eye, head, muscle);

        % get quaternion from rotation matrix
        q_eye = rotm2quat(R_eye_in_world);
        q_head = rotm2quat(R_head_in_world);

        history_tau_eye(i,:) = eye_torque;
        history_tau_head(i,:) = head_torque;

        history_state(i,:) = [q_eye, q_head , omega_eye', omega_head'];

    end

end

final_state = [q_eye, q_head , omega_eye', omega_head'];


% End of physics computation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


if flags.visual
    %%%%%% Show Graphical Representation %%%%%%
    %First run rviz and open the config file in using_markers/rviz if you want
    %visualization.
    %set function for visualization
    if flags.wrapping == 0
        visualization_handle = @rviz_visuals;
    else
        visualization_handle = @rviz_visuals_wrap;
    end
    visualization_handle(p, history_state,history_delta_l, history_slack, muscle)
    rosshutdown
end


if flags.plot
    %%%%%% Plots %%%%%%
    x = linspace(0,i,i);

    % Force magnitudes
    figure();
    plot(x,history_f_eye(:,1),'Color',[0,0.4470,0.7410]);
    grid on
    grid minor
    hold on
    plot(x,history_f_eye(:,2),'Color',[0.8500 0.3250 0.0980]);
    plot(x,history_f_eye(:,3),'Color',[0.9290 0.6940 0.1250]);
    plot(x,history_f_eye(:,4),'Color',[0.4940 0.1840 0.5560]);
    plot(x,history_f_eye(:,5),'Color',[0.4660 0.6740 0.1880]);
    plot(x,history_f_eye(:,6),'Color',[0.3010 0.7450 0.9330]);
    xlabel('time');
    ylabel('muscle forces');
    legend("IR","MR","SR","LR","IO","SO");

    % Eye orientation in euler angles
    figure();
    hold on
    plot(x,quat2eul(history_state(:,1:4)));
    grid on
    grid minor
    xlabel('time(ms)');
    ylabel('eye orientation(rad)');
    legend("eye theta x", "eye theta y", "eye theta z");

    % Eye velocities
    figure();
    hold on
    plot(x,history_state(:,9:11));
    grid on
    grid minor
    xlabel('time(ms)');
    ylabel('eye velocity(rad/s)');
    legend("eye omega x", "eye omega y", "eye omega z");

    % Head velocities
    figure();
    plot(x,history_state(:,12:14));
    grid on
    grid minor
    xlabel('time(ms)');
    ylabel('head angular velocity (rad/s)');
    legend("head omega x", "head omega y", "head omega z");

    % % Eye torques
    % figure();
    % hold on
    % plot(x,history_tau_eye);
    % grid on
    % grid minor
    % xlabel('time(ms)');
    % ylabel('eye torque (N.m)');
    % legend("torsion", "horizontal", "vertical");

    % % Head torques
    % figure();
    % plot(x,history_tau_head);
    % grid on
    % grid minor
    % xlabel('time(ms)');
    % ylabel('head torque (N.m)');
    % legend("torsion", "horizontal", "vertical");
end

end