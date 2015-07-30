function [x, P] = updateIEKF(x, P, z, Rf, idf, N)
% Inputs:
%   z, R - range-bearing measurements and covariances
%   idf - feature index for each z
%   N - number of iterations of the IEKF
%
% Outputs:
%   x, P - updated state and covariance (globals)
%%
    if isempty(idf), return; end
    lenz     = size(z,2);
    RIterate = zeros(2 * lenz); 
    zIterate = zeros(2 * lenz, 1);
    for i = 1:lenz
        j             = 2 * i + (-1:0);    
        zIterate(j)   = z(:,i);
        RIterate(j,j) = Rf(:,j);
    end      
    [x,P] = iterate(x, P, zIterate, RIterate, ...
        @hmodel, @hjacobian, idf, N); 

    function v = hmodel(x, z, idf)
    lenz = length(idf);
    v    = zeros(2 * lenz, 1);
    for i= 1:lenz
        j       = 2 * i + (-1:0);    
        [zp,H]  = obsModel(x, idf(i));
        v(j)    = z(j)-zp;
    end


    function H = hjacobian(x, idf)
    lenz = length(idf);
    lenx = length(x);
    H    = zeros(2 * lenz, lenx);

    for i = 1:lenz
        j           = 2 * i + (-1:0);
        [zp,H(j,:)] = obsModel(x, idf(i));
    end
