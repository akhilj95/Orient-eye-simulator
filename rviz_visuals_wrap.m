function [] = rviz_visuals_wrap(p, history_state,history_delta_l, history_slack,muscle)
% Not implemented yet
% Currently same as rviz_visuals

points = zeros(1,70);
arrow_end = [0.515 0 0]';
goal_end = [0.01 0 0]';
eq_end = [0.01 0 0]';


rosinit();
trajectory = zeros(1,7);
chatpub = rospublisher('/insertion_points','std_msgs/Float64MultiArray');
chatpub1 = rospublisher('/trajectory2','geometry_msgs/Pose');
msg = rosmessage(chatpub);
msg1 = rosmessage(chatpub1);

pause(0.5);
for i=1:p
    insert1 = quat2rotm(history_state(i,1:4))*muscle.E;
    arrow = quat2rotm(history_state(i,1:4))*arrow_end;

    for j = 1:6
        points(6*(j-1)+1:6*(j-1)+3) = insert1(:,j);
        points(6*(j-1)+4:6*(j-1)+6) = muscle.H(:,j);
        len = norm(insert1(:,j)-muscle.H(:,j));
        ratio_d = history_delta_l(i,j)/len;
        points(46+3*(j-1):48+3*(j-1)) = ratio_d*insert1(:,j) + (1-ratio_d)*muscle.H(:,j);
    end
    points(37:39) = arrow;
    trajectory(1:4) = history_state(i,1:4);
    trajectory(5:7) = 1.3*arrow;
    points(40:45) = history_slack(i,:);
    points(64:66) = goal_end;
    points(68:70) = eq_end;
%     if (i == p-50)
%         points(67) = 2;
%         pause(0.5);
%     end
    if (norm(history_state(i,9:11)) > 0.005)
         pause(0.002);
    end

    msg.Data = points;
    msg1.Orientation.X = trajectory(1);
    msg1.Orientation.Y = trajectory(2);
    msg1.Orientation.Z = trajectory(3);
    msg1.Orientation.W = trajectory(4);
    msg1.Position.X = trajectory(5);
    msg1.Position.Y = trajectory(6);
    msg1.Position.Z = trajectory(7);
    send(chatpub,msg);
    points(67) = 0;
    send(chatpub1,msg1);
end
    points(67) = 1;
    msg.Data = points;
    send(chatpub,msg);


end

