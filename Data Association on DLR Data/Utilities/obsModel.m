function [z, H] = obsModel(x, idf)
   %%
    Nxv  = 3;
    fpos = Nxv + idf * 2 - 1; % Order of observations in state variance.
    H    = zeros(2, length(x));

    % Auxiliary values
    dx  = x(fpos)  -x(1); 
    dy  = x(fpos+1)-x(2);
    c   = cos(x(3)); 
    s   = sin(x(3));
    rot = [ c s;
           -s c];% coordinate transformation matrix
    %% Predict measurements.
    z =  rot * [dx ; dy]; %transform from world coordinate to vehicle-based coordinate
    %% Calculate H
    H(:,1:3)         = [-c  -s  -s*dx+c*dy;   %Jacobian to vehicle pose
                         s  -c  -c*dx-s*dy];
    H(:,fpos:fpos+1) = [ c   s;   -s   c];    %Jacobian to feature
    
end

