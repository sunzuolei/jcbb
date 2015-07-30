function [idn, zn, Rn, idf, zf, Rf, T, M] = DA(x, P, z, R, obsId, mapId, chi2)
% idf - index of observations.Determine the position in state.
% Use nearest neighbour or jonint compatibility to conduct data association.
Nxv = 3; % number of vehicle pose states
Nf  = (length(x) - Nxv)/2; % number of features already in map
zf  = [];  zn  = [];
Rf  = [];  Rn  = [];
idf = [];  idn = [];
%% There are no observation measurements from sensor
if size(z,2)==0
    M  = []; 
    T  = [];
    disp('Get no observation measurements at this step!');
    return;
end
%% If no features in map already£¬add the new observations into the map.
if Nf == 0
  for i   = 1:size(z,2)
      zn  = [zn z(:,i)];
      Rn  = [Rn R(:,:,i)];
      idn = [idn obsId(i)];
  end
 M  = zeros(1,size(z,2));
 T  = zeros(1,size(z,2));
else
%% Prepare for data association
predictFeature = predictFeatures (x, Nf,P);%compute predicted features 

%% chose associate method ICNN or JC
[M, compatibility ]   = ICNNModule(predictFeature, chi2, z, R, Nf);
% [M, compatibility] = JCModule(predictFeature, chi2, z, R, Nf);

%% Classify observations according to associate result M
   %R needs to change with different data sets 
 for i  = 1:size(z,2)
       ii = obsId(i);
     if (M(i) ~= 0) %match with map feature
        zf  = [zf  z(:,i)];
        idf = [idf M(i)];
        Rf = [Rf R(:,:,i)];
     elseif compatibility.AL(i)==0
        zn   = [zn z(:, i)];
        Rn = [Rn R(:,:,i)];
        idn  = [idn ii];
     end
 end
 %% Display results for data association 
 T=[];
  for i = 1:size(z,2)
       f = find(mapId == obsId(i));
       if length(f)>=1
          T = [T f(1)];
       else
          T = [T 0];
       end
   end
disp('------------------------------------------------------------------------------------');
disp(['GROUND  TRUTH: ' sprintf('%2d  ', T)]);
    disp(['MY HYPOTHESIS: ' sprintf('%2d  ', M)]);
disp(['Correct (1/0)? ' sprintf('%2d  ', T == M)]);
disp(' ');
pause(0.5);
end
end

function pre = predictFeatures (x, Nf,P)
pre.z   = zeros(2*Nf, 1);
pre.H   = zeros(2*Nf, length(x));
pre.HPH = zeros(2*Nf);
P=symmetric(P);
for j = 1:Nf
    i = 2*j + (-1:0);
    [zp, H]   = obsModel(x, j);
     pre.z(i) = [zp(1) ; zp(2)];
     pre.H(i,:) = [H(1,:); H(2,:)];
end
pre.HPH =  pre.H * P * pre.H';
end

function a=symmetric(b)
a=(b+b')*0.5;
end