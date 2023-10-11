function tangent_points=get_tangent_points(insertion_head,insertion_eye,eye_radius)
% Needs the input insertion points to be in size (3 x no.of points matrices) with each point 
% as a column vector

% Origin of the coordiante frame is taken at centre of eye.
% The cable can be considered to be in the plane containing the origin, the 
% insertion point on head and the insertion point on eye if we assume there is no
% friction.

n = cross(insertion_head,insertion_eye); % normal vector to the plane for each cable
n_unit_vec= n./vecnorm(n);
rho= eye_radius./vecnorm(insertion_head); % we divide by norm of x1 here to simplify calc
k= cross(n_unit_vec,insertion_head);% vector perpendicular to x1
tangent_points= (rho.^2).*insertion_head + rho.*(sqrt(1-rho.^2)).*k;

end