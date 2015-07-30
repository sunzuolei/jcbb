function result = resultStore (result, z, GT, M)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% University of Zaragoza
% Centro Politecnico Superior
% Robotics and Real Time Group
% Authors:  J. Neira, J. Tardos
% Date   :  7-2004
%-------------------------------------------------------
%-------------------------------------------------------

total = size(z,2);%每一步观测值的个数
tp = sum((M & (M == GT)));
tn = sum((not (M) & (M == GT)));
fp = sum(M & not (M == GT));
fn = sum(not (M) & not (M == GT));

result.total = [result.total total];
result.tp = [result.tp tp];
result.tn= [result.tn tn];
result.fp = [result.fp fp];
result.fn = [result.fn fn];

% if (not(min(M == GT)))
%     disp('Hypothesis not in agreement with ground truth');
%     disp(sprintf('True positives: %d', tp));
%     disp(sprintf('True negatives: %d', tn));
%     disp(sprintf('False positives: %d', fp));
%     disp(sprintf('False negatives: %d', fn));
    %wait;
end

