function [arc_points,alpha] = get_arc_angle(t_eye,insertion_eye,n)
% can be shortened to have less than 180 degrees
arc_points = zeros(3,n,6);
for i=1:6
   u = [t_eye(1,i) t_eye(2,i) t_eye(3,i)];
   v = [insertion_eye(1,i) insertion_eye(2,i) insertion_eye(3,i)];
   cos_alpha = (u(1)*v(1) + u(2)*v(2) + u(3)*v(3))/((norm(u))*norm(v));
   sen_alpha = (norm(cross(u,v)))/(norm(u)*norm(v));
   if cos_alpha>=0 && sen_alpha>=0
       alpha(i) = asin(sen_alpha);
   elseif cos_alpha<=0 && sen_alpha>=0
       alpha(i) = acos(cos_alpha);
   elseif cos_alpha<0 && sen_alpha<0
       alpha(i) = 2*pi-acos(cos_alpha);
   elseif cos_alpha>=0 && sen_alpha <=0
       alpha(i) = asin(sen_alpha) + 2*pi;
   end
   
   % points on the arc
   for j=1:n
       theta=j*alpha(i)/n;
       arc_points(1,j,i)=(sin(alpha(i)-theta)*u(1)+sin(theta)*v(1))/sin(alpha(i));
       arc_points(2,j,i)=(sin(alpha(i)-theta)*u(2)+sin(theta)*v(2))/sin(alpha(i));
       arc_points(3,j,i)=(sin(alpha(i)-theta)*u(3)+sin(theta)*v(3))/sin(alpha(i));
   end
end
