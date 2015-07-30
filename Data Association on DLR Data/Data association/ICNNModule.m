function [M, compatibility] = ICNNModule( predictFeature, chi2, z, R, Nf)
% Use nearest neighbour algorithm to complete data association.

%% Comput Mahalanobis distance for each pair of 
 % a sensor measurement and a map feature
MD2 = computeMD2 (Nf, z, predictFeature, R);
compatibility.IC = MD2 < chi2(2);
%The IC principle
compatibility.AL = (sum (compatibility.IC,2))';
%The aim of this step is to rule out false alarm.
M      = zeros(1,size(MD2,1));

%% find nearest neighbour match and do IC test 
for i = 1:size(MD2,1)
    MD2min = Inf;
    for j = 1:size(MD2,2)
        if MD2(i,j) < MD2min
           MD2min   = MD2(i,j);
           nearest  = j;
        end
    end
    if MD2min < chi2(2)
       M(i)   = nearest;        
    end
end
end

function MD2 = computeMD2(Nf, z, pre, R)
%Here pre notes the prediction of features
MD2 = zeros(length(z), Nf);
%MD2 is a matrix whose rows are length(z) and colums are Nf.
z   = reshape(z,[],1);
for i  = 1:length(z)/2
    ii = 2*i + (-1:0);
    zi =  z(ii); 
    Ri =  R(:, :, i);
     for j = 1:Nf
         jj = 2*j + (-1:0);
         zp = pre.z(jj);%The predict value of  map feature
         v = zi-zp;
         S = pre.HPH(jj,jj) + Ri; 
         MD2(i, j) = mahalanobis(v, S);%squared Mahalanobis distance for each pair
     end
end
end

function d2 = mahalanobis (z, P)
Cp = chol(P);
y = Cp'\z;
d2 = full(y'*y);
end
