% Calibrates accelerometers, joint encoders using only self sensors
% measurements. this application calibrates:
% - accelerometers offsets/gains (full matrix 3x3)
% - joint encoders offsets
% - head IMU embedded accelerometers
%

% Add main folders in Matlab path
run generatePaths.m;

%% clear all variables and close all previous figures
clear
close all
clc

%Clear static data
clear classes;
clear functions;

% Clear all timers
System.clearTimers();

% Create YARP Network device, for initializing YARP classes for communication
yarp.Network.init();

% load application main interface parameters
init = Init.load('sensorSelfCalibratorInit');
% overwrite init parameters in the case of repeatability tests (refer to script `testAccelCalibratorRepeatability.m`)
global repeatabilityTestSeqNum;
if ~isempty(repeatabilityTestSeqNum)
    if (repeatabilityTestSeqNum == -1)
        init.accelerometersCalib.sensorDataAcq = {'new'};
    else
        init.accelerometersCalib.sensorDataAcq = {'seq',repeatabilityTestSeqNum};
    end
end

% Create robot model. The model holds the robot name, the parameters
% extracted from the URDF model, the sensor calibration parameters and the
% joint/motor parameters (PWM to torque rate, friction parameters, ...).
model = RobotModel(init.modelName,init.modelPath,init.calibrationMapFile);

% Load last acquired data accessors from file
if exist('lastAcqSensorDataAccessorMap.mat','file') == 2
    load('lastAcqSensorDataAccessorMap.mat','lastAcqSensorDataAccessorMap');
end
if ~exist('lastAcqSensorDataAccessorMap','var')
    lastAcqSensorDataAccessorMap = containers.Map('KeyType','char','ValueType','any');
end

% All below procedures are optional and checked/unchecked in the main
% interface parameters

isTaskScheduled = [...
    init.calibrateAccelerometers,...
    init.calibrateJointEncoders,...
    init.acquireSensorsTestData,...
    init.calibrateFTsensors,...
    init.calibrateGyroscopes,...
    init.calibrateLowLevTauCtrl,...
    init.calibrateLowLevCurrCtrl];

calibratorTasks = {...
    @AccelerometersCalibrator.instance,...
    @JointEncodersCalibrator.instance,...
    [],...
    [],...
    [],...
    @LowlevTauCtrlCalibrator.instance,...
    @LowlevCurrCtrlCalibrator.instance};

% filter activated tasks and parameters
calibratorTasks = calibratorTasks(isTaskScheduled);

%% 1 - Run the calibrators: data acquisition, processing and diagnosis
% 

for cTaskAccessor = calibratorTasks
    % unwrap task accessor and get the calibrator singleton
    task = cTaskAccessor{1}();
    task.run(init,model,lastAcqSensorDataAccessorMap);
end


%% Uninitialize yarp
yarp.Network.fini();


%%
%%===================================================================================
% Static local functions
%%===================================================================================

% ...
