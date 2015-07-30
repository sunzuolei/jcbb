dbstop if error;
clear all; close all;
path(path, genpath('../Unknown Data Association of IEKF and EKF SLAM on DLR Data'));
load 'Data/truth.mat';
chi2=chi2inv(0.99,1:1000); % gain table for chi-square test.
%% switches
visualize = 1;
openIEKF = 0; % zero: run EKF-SLAM demo. Nonzero: run IEKF-SLAM,
              % and the value of openIEKF is the iterate number.
%% parametres configuration
load 'Data/relMotion.mat'; % robot odometry data in DLR
load 'Data/landmark.mat';  % observations in DLR.
step     = 3297;
fprintf('Total step: %d\n', step);
pos      = truth.x(:,1);             % robot initial pose.
%   pos(3)   = pos(3)+0.065 ;         % modify the pose.
cov      = truth.P(:,:,1);           % Initial covariance.
%% parameters for association result visualization  
M      = [];% hyphothesis for matching of observations and features
GT     = [];% ground truth for pairing of observation and features
mapId  = [];%  
result.total = []; result.fp    = [];
result.tp    = []; result.fn    = [];
result.tn    = []; % assign memory for association result  
    %% assign memory and Initialise.
    data.path      = zeros(3, step+1); % assign memory for estimated path.
    data.path(:,1) = pos;
    data.pos(1).x  = pos; % state include robot and landmarks.
%     data.pos(1).P = diag(cov);
    %% main Loop
    disp('Wait for a moment...');
    tic;
   for i = 1:step  %EKF recursive
   fprintf('Step: %d\n', i);
         %% predict simulation
         [pos, cov] = predictEKF(pos, cov, relMotion(i).x, relMotion(i).P);     
         %% oberserve each step and conduct data association
         [idn, zn, Rn, idf, zf, Rf, T, M ] = DA(pos, cov, landmark(i).z, landmark(i).R, landmark(i).ID, mapId, chi2);                 
         result = resultStore(result, landmark(i).z, T, M);                                   
         %% update 
          if openIEKF == 0
                [pos, cov] = updateEKF(pos, cov, zf, Rf, idf);
          else
                [pos, cov] = updateIEKF(pos, cov, zf, Rf, idf, openIEKF);
          end
          [pos, cov, mapId] = augmentState(pos, cov, zn, Rn, idn, mapId);
         %% store data
         data.path(:,i+1) = pos(1:3); % convenient for animation
         data.pos(i+1).x  = pos;
%          data.pos(i+1).P = diag(cov);
    end
    toc;
%% animation.
vis(data, truth, step, visualize, openIEKF);
showResult(result,step);

        
        