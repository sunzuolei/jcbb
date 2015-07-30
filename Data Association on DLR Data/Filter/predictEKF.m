function [x, P] = predictEKF(x, P, dr, Q)
    % Inputs:
    %   x,P - robot and landmarks's state and their covariance
    %   dr  - control variance
    %   Q   - process noise covariance
    % Outputs:
    %   x, P - global state and covariance
    %% Auxiliary values
    s= sin(x(3)); c= cos(x(3));

    % Jacobians   
    Fxv= [1 0 (-s*dr(1)-c*dr(2));
          0 1  (c*dr(1)-s*dr(2));
          0 0                 1];
    Fdr= [c -s 0;
          s  c 0;
          0  0 1]; 

    %% Predict robot pose.
    x(1:3)= [x(1) + c*dr(1)-s*dr(2);
             x(2) + s*dr(1)+c*dr(2);
             piTopi(x(3) +   dr(3))];
    %% Predict covariance
    P(1:3,1:3)= Fxv*P(1:3,1:3)*Fxv' + Fdr*Q*Fdr';
    if size(P,1)>3
        P(1:3,4:end)= Fxv*P(1:3,4:end);
        P(4:end,1:3)= P(1:3,4:end)';
    end
 end