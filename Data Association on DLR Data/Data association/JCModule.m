function [M,compatibility] = JCModule( predictFeature, chi2, z, R, Nf)
%Use joint compatibility algorithm to complete data association.

MD2 = computeMD2 (Nf, z, predictFeature, R);
%MD2 is matrix,its elements are squared Mahalanobis distance.
compatibility.IC = MD2 < chi2(2);
compatibility.AL = (sum (compatibility.IC,2))';

N = JCTest(predictFeature, z, R, compatibility, [], 1, chi2);
M = N;
end

function  N = JCTest(pre, z, R, compatibility, M, i, chi2)
N = [];
lenz = size(z,2);
if i >lenz
    N = M;
else
    individuallyCompatible = find(compatibility.IC(i, :));     
        for j = individuallyCompatible            
            if  (jointlyCompatible(pre, z, R, [M j], chi2))
                N = JCTest(pre, z, R, compatibility, [M j], i + 1, chi2); %pairing (Ei, Fj) accepted 
                break;
            end
        end
        if pairings(M) + pairings(compatibility.AL(i+1:end)) >= pairings(N)%match for IC but not fits JC, move to next z 
         N = JCTest(pre, z, R, compatibility, [M 0], i + 1, chi2);
        end
end
end 

function [answer, JMD2] = jointlyCompatible (pre, z, R, N, chi2)
JMD2 = jointMahalanobis2 (pre, z, N, R);
dof = 2*length(find(N));
%dof is the number of freedom.
% dof = 2*observations.m;
answer = JMD2 < chi2(dof);
%JC test principle.
end

function JMD2 = jointMahalanobis2 (pre, z, N, R)
% Compute squared joint Mahalanobis distance for a hypothesis
z   = reshape(z,[],1);
k   = find(N);
Hi  = []; zi = [];
Ri  = []; zp = [];
HPH =[];
for i  = k
    ii = 2*i+(-1:0);
    zi = [zi; z(ii)];
    t  = length(Ri);
    tt = t+(1:2);
    Ri(tt, tt) =  R(:,:,i);        
    j  = N(i);
    jj = 2*j+(-1:0);
    zp = [zp; pre.z(jj)];
    HPH(tt,tt) = pre.HPH(jj,jj);
end
v = zi - zp;
S = HPH + Ri;
JMD2 = mahalanobis(v, S);
%Here the mahalanobis distance is joint mahalanobis distance.
end

function MD2 = computeMD2(Nf, z, pre, R)
lenz=size(z,2);
MD2 = zeros(lenz, Nf);
z   = reshape(z,[],1);
for i  = 1:lenz
    ii = 2*i + (-1:0);
    zi =  z(ii); 
    Ri =  R(:, :, i);
     for j = 1:Nf
         jj = 2*j + (-1:0);
         zp = pre.z(jj);
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

function p = pairings(M)
%Here p is different from the p of ICNN,it is related to pairs of M.
p = length(find(M));
end