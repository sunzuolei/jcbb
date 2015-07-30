function Xwb = compound(Xwa, Xab)
    % 
    % For details, please refer to Section 3.2.1 of R. Smith, M. Self, 
    % "Estimating uncertain spatial relationships in robotics," 
    % Autonomous Robot Vehicles, pp. 167-193, 1990. 
    %   Xwa - should be of size 3*1 or 1*3
    %   Xab - can be 3*n or 2*n matrix
    % Seeded by Tim Bailey
    % Modified by Samuel on 7 May 2013

   rot = [cos(Xwa(3)-pi/2)    -sin(Xwa(3)-pi/2); 
          sin(Xwa(3)-pi/2)     cos(Xwa(3)-pi/2)];
   
   Xwb(1:2,:) = rot * Xab(1:2,:) + repmat(Xwa(1:2), 1, size(Xab,2));

    %% if Xwb is a pose and not a point
    if size(Xab,1) == 3
       Xwb(3,:) = piTopi(Xwa(3,:) + Xab(3));
    end
end