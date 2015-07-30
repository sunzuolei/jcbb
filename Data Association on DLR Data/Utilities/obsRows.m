function [ ix,iy,indi ] = obsRows( z )
%OBSROWS Summary of this function goes here
%   Detailed explanation goes here
ix = 2 * z -1;
iy = 2 * z;
indi = reshape ([ix;iy],[],1);
end

