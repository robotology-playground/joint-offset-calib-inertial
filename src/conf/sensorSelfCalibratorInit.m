%====================================================================
% This configuration file defines the main application parameters
%====================================================================

%% Common model and calibration input parameters
modelName = 'iCubGenova04'; % as the models defined in 'icub-models' repo
dataPath  = '../../data/dumper';
%modelPath = '../models/icubSim/icub.urdf';
modelPath = '../models/iCubGenova04/model.urdf';
%calibrationMapFile = '../../data/calibration/calibrationMap_#6.mat';
calibrationMapFile = 'calibrationMap.mat';
%calibrationMapFile = '';
isModelOnline = RobotModel.isOnline;
isModelOnline.value = true;

%% Standard or custom calibration
calibrationType = 'standard';

%% standard calibration tasks checklist
acquireSensorsTestData  = false;
calibrateAccelerometers = false;
calibrateJointEncoders  = false;
calibrateFTsensors      = false;
calibrateGyroscopes     = false;
calibrateLowLevTauCtrl  = true;

%% Diagnosis and visualization
runDiagnosis = false;

%% Common output parameters
saveCalibration = true;
defaultSavePlot = true;
defaultExportPlot = true;


%% Custom calibration sequence
% the below structured list defines the tasks sequence and their set of
% parameters.
% 
% customCalibSequence = {...
%     'acquire'    ,{'seqCfg1','seqCfg2',...,'seqCfgN'};...
%     'diag'       ,{'seqCfg1','seqCfg2',...,'seqCfgN'};...
%     'calibAccs'  ,'seqCfgX';...
%     'calibJoints','seqCfgY';...
%     'saveCalib',
%     };


%% 'acquireSensorsTestData': Acquire only sensors test data (only accelerometers for now)

% define the robot limb holding the sensors on which we run the diagnosis.
acquiredParts = {'torso'};
% Profile = ... TBD!!

% Fine selection of the accelerometers:
% Select (set to 'true') the accelerometers to analyse through the respective indexes.
% 
% (Part)           (inertial MTB boards)
% 
%                  (upper)       (forearm)      (hand)
% left_arm :       1b10..1b13    1b8..1b9       1b7
% right_arm:       2b10..2b13    2b8..2b9       2b7
% mtbSensorAct:      10....13      8....9         7
%                  (upper)       (lower)        (foot)
% left_leg :       10b1..10b7    10b8..10b11    10b12,10b13
% right_leg:       11b1..11b7    11b8..11b11    11b12,11b13
% mtbSensorAct idx:   1.....7       8.....11       12,   13
% 
% head     :       head_imu 3D accelerometer
% mtbSensorAct idx:  1
% 
% torso    :       0b7..0b10
% mtbSensorAct idx:  7....10
% 
% some sensors are de-activated because of faulty behaviour
mtbSensorAct.left_arm = [10:13 8:9 7];
mtbSensorAct.right_arm = [10:13 8:9 7];
mtbSensorAct.left_leg = 1:13;
mtbSensorAct.right_leg = 1:13;
mtbSensorAct.torso = 7:10;
mtbSensorAct.head = 1;

% Save generated figures: if this flag is set to true, all data is saved and figures 
% printed in a new folder indexed by a unique iteration number. Log
% classification information are saved in text format for easier search from
% a file explorer.
savePlot = defaultSavePlot;
exportPlot = defaultExportPlot;
loadJointPos = true;

% Motion sequence profile
%motionSeqProfile = 'jointsCalibratorSequenceProfile';
%motionSeqProfile = 'accelerometersCalibratorSequenceProfileWOsuspend';
motionSeqProfile = 'gyroscopesCalibratorSequenceProfile2checkCalib';
%motionSeqProfile = 'gyroscopesCalibratorSequenceProfile1checkAlgo';

% Wrap parameters specific to calibrator or diagnosis functions processing
taskSpecificParams = struct(...
    'mtbSensorAct',mtbSensorAct,...
    'savePlot',savePlot,...
    'exportPlot',exportPlot,...
    'loadJointPos',loadJointPos,...
    'motionSeqProfile',motionSeqProfile);

% Sensor data acquisition: ['new'|'last'|<id>]
sensorDataAcq = {'seq',53};

% wrap parameters ('acquiredParts' renamed as 'calibratedParts' because this is handled as
% a calibrator task)
sensorsTestDataAcq = struct(...
    'calibedParts',{acquiredParts},...
    'taskSpecificParams',taskSpecificParams,...
    'sensorDataAcq',{sensorDataAcq});

clear acquiredParts mtbSensorAct savePlot exportPlot loadJointPos ...
    sensorDataAcq motionSeqProfile taskSpecificParams;

%% 'calibrateAccelerometers': MTB/IMU accelerometers gains/offsets calibration

% Calibrated parts:
% Only the accelerometers from these parts (limbs) will be calibrated
calibedParts = {'torso','head'};

% some sensors are de-activated because of faulty behaviour, bad calibration 
% or wrong frame definition
mtbSensorAct.left_arm = [10:13 8:9 7];
mtbSensorAct.right_arm = [10:13 8:9 7];
mtbSensorAct.left_leg = 1:13;
mtbSensorAct.right_leg = 1:13;
mtbSensorAct.torso = 7:10;
mtbSensorAct.head = 1;

% Save generated figures: if this flag is set to true, all data is saved and figures 
% printed in a new folder indexed by a unique iteration number. Log
% classification information are saved in text format for easier search from
% a file explorer.
savePlot = defaultSavePlot;
exportPlot = defaultExportPlot;
loadJointPos = false;

% Wrap parameters specific to calibrator or diagnosis functions processing
taskSpecificParams = struct(...
    'mtbSensorAct',mtbSensorAct,...
    'savePlot',savePlot,...
    'exportPlot',exportPlot,...
    'loadJointPos',loadJointPos);

% Sensor data acquisition: ['new'|'last'|<id>]
sensorDataAcq = {'last'};

% wrap parameters
accelerometersCalib = struct(...
    'calibedParts',{calibedParts},...
    'taskSpecificParams',taskSpecificParams,...
    'sensorDataAcq',{sensorDataAcq});

clear calibedParts mtbSensorAct savePlot exportPlot loadJointPos ...
    sensorDataAcq taskSpecificParams;

%% 'calibrateJointEncoders' Joint encoders offsets calibration

% Calibrated parts:
% Only the joint encoders from these parts (limbs) will be calibrated
calibedParts = {'torso'};

% Fine selection of joint encoders:
% Select the joints to calibrate through the respective indexes. These indexes match 
% the joint names listed below, as per the joint naming convention described in 
% 'http://wiki.icub.org/wiki/ICub_Model_naming_conventions', except for the torso.
%
%      shoulder pitch roll yaw   |   elbow   |   wrist prosup pitch yaw   |  
% arm:          0     1    2     |   3       |         4      5     6     |
%
%      hip      pitch roll yaw   |   knee    |   ankle pitch  roll       |  
% leg:          0     1    2     |   3       |         4      5          |
%
%               yaw   roll pitch |
% torso:        0     1    2     |
%
%               pitch roll yaw   |
% head:         0     1    2     |
%
%=================================================================
% !!! Below joint indexes will be ignored for parts not defined in
% 'calibedParts' !!!
%=================================================================
calibedJointsIdxes.left_arm = 0:3;
calibedJointsIdxes.right_arm = 0:3;
calibedJointsIdxes.left_leg = 0:5;
calibedJointsIdxes.right_leg = 0:5;
calibedJointsIdxes.torso = 0:2;
calibedJointsIdxes.head = 0:2;

% some sensors are de-activated because of faulty behaviour, bad calibration 
% or wrong frame definition
mtbSensorAct.left_arm = [10:13 8:9 7];
mtbSensorAct.right_arm = [10:13 8:9 7];
mtbSensorAct.left_leg = 1:13;
mtbSensorAct.right_leg = 1:13;
mtbSensorAct.torso = 7:10;
mtbSensorAct.head = 1;

% Save generated figures: if this flag is set to true, all data is saved and figures 
% printed in a new folder indexed by a unique iteration number. Log
% classification information are saved in text format for easier search from
% a file explorer.
savePlot = defaultSavePlot;
exportPlot = defaultExportPlot;
loadJointPos = true;

% Wrap parameters specific to calibrator or diagnosis functions processing
taskSpecificParams = struct(...
    'calibedJointsIdxes',calibedJointsIdxes,...
    'mtbSensorAct',mtbSensorAct,...
    'savePlot',savePlot,...
    'exportPlot',exportPlot,...
    'loadJointPos',loadJointPos);

% Sensor data acquisition: ['new'|'last'|<id>]
sensorDataAcq = {'seq',53};

% wrap parameters
jointEncodersCalib = struct(...
    'calibedParts',{calibedParts},...
    'taskSpecificParams',taskSpecificParams,...
    'sensorDataAcq',{sensorDataAcq});

clear calibedParts calibedJointsIdxes mtbSensorAct savePlot exportPlot loadJointPos ...
    sensorDataAcq taskSpecificParams;

%% 'calibrateLowLevTauCtrl' Joint low level torque control parameters calibration

% Calibrated parts:
% Only the joint parameters from these parts (limbs) will be calibrated
calibedParts = {'right_leg'};

% Fine selection of joints to calibrate:
% Select the joints to calibrate through the respective indexes. These indexes match 
% the joint names listed below, as per the joint naming convention described in 
% 'http://wiki.icub.org/wiki/ICub_Model_naming_conventions', except for the torso.
%
%      shoulder pitch roll yaw   |   elbow   |   wrist prosup pitch yaw   |  
% arm:          0     1    2     |   3       |         4      5     6     |
%
%      hip      pitch roll yaw   |   knee    |   ankle pitch  roll       |  
% leg:          0     1    2     |   3       |         4      5          |
%
%               yaw   roll pitch |
% torso:        0     1    2     |
%
%               pitch roll yaw   |
% head:         0     1    2     |
%
%=================================================================
% !!! Below joint indexes will be ignored for parts not defined in
% 'calibedParts' !!!
%=================================================================
calibedJointsIdxes.left_arm = 0:3;
calibedJointsIdxes.right_arm = 0:3;
calibedJointsIdxes.left_leg = 0:5;
calibedJointsIdxes.right_leg = 0:5;
calibedJointsIdxes.torso = 0:2;
calibedJointsIdxes.head = 0:2;

% Save generated figures: if this flag is set to true, all data is saved and figures 
% printed in a new folder indexed by a unique iteration number. Log
% classification information are saved in text format for easier search from
% a file explorer.
savePlot = defaultSavePlot;
exportPlot = defaultExportPlot;

% Wrap parameters specific to calibrator or diagnosis functions processing
taskSpecificParams = struct(...
    'calibedJointsIdxes',calibedJointsIdxes,...
    'savePlot',savePlot,...
    'exportPlot',exportPlot);

% Sensor data acquisition: ['new'|'last'|<id>]
sensorDataAcq = {'new'};

% wrap parameters
lowLevelTauCtrlCalib = struct(...
    'calibedParts',{calibedParts},...
    'taskSpecificParams',taskSpecificParams,...
    'sensorDataAcq',{sensorDataAcq});

clear calibedParts calibedJointsIdxes savePlot exportPlot loadJointPos ...
    sensorDataAcq taskSpecificParams;

%% FT sensors gains/offsets calibration (TBD)

ftSensorsCalib = struct();
gyroscopesCalib = struct();

