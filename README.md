# jcbb
The demo of loop closure detecting with JCBB algorithm

**Note:**

Before run the script "run.m", you should set the path by hand. In the Matlab menu, just click Home->Set path>Add folder, and select the path of the file "IEKF and EKF SLAM on DLR data".

**The following files mean :**
## Data
[DLR data set](http://www.informatik.uni-bremen.de/agebv/en/DlrSpatialCognitionDataSet) and experimental data.
* **truth.mat :** ground truth in DLR data set.
* **relMotion.mat :** odometry in DLR data set.
* **landmark.mat :** observations in DLR data set.
* **EKFobs4.mat :** experimental data of EKF-SLAM.
* **IEKF2obs4.mat :** experimental data of IEKF-SLAM.

## Data association
Codes for data association in SLAM.
* **ICNNModule.m :** individual compatibility nearest neighbor algorithm.
* **JCModule.m :** joint compatibility branch and bound algorithm.
* **DA.m :** codes for data association ,you can use ICNN or JCBB.

## Filter
It contains the filter codes for SLAM execution.
* **predictEKF.m :** predict step of EKF and IEKF SLAM. 
* **updateEKF.m :** update step of EKF-SLAM.
* **updateIEKF.m** and **iterate.m :** update step of IEKF-SLAM.
* **augmentState.m :** augment step, that is add new features to the map.

## Utilities
Models and tools for SLAM demo. 
* **compound.m :** complete the estimating uncertain spatial relationships in robotics.
* **corresponding.m :** classify features with known corresponding.
* **piTopi.m :** make sure the angle is in [-pi, pi].
* **obsModel.m :** observation model.
* **vis.m :** for animation.