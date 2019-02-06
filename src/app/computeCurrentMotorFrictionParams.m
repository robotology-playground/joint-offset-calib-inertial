% CONVERT CURRENT TO JOINT TORQUE MOTOR AND FRICTION PARAMETERS
if false
% Add main folders in Matlab path
run generatePaths.m;

%% clear all variables and close all previous figures
clear
close all
clc

System.clearTimers()
%Clear static data
clear classes;
clear functions;

% Clear all timers
System.clearTimers();

% Create YARP Network device, for initializing YARP classes for communication
yarp.Network.init();

% load model input parameters
modelName = 'iCubGenova04'; % as the models defined in 'icub-models' repo
modelPath = '../models/iCubGenova04/model.urdf';
calibrationMapFile = 'calibrationMap-305to398.mat';
calibrationMapFile2 = 'calibrationMap-305to398.mat';
isModelOnline = RobotModel.isOnline;
isModelOnline.value = false;

% Create robot model. The model holds the robot name, the parameters
% extracted from the URDF model, the sensor calibration parameters and the
% joint/motor parameters (PWM to torque rate, friction parameters, ...).
model = RobotModel(modelName,modelPath,calibrationMapFile);

% Convert old parameters from ElectricalMotorTransFunc
load(calibrationMapFile,'calibrationMap');

for motorName = calibrationMap.keys
    calib = calibrationMap(motorName{1});
    calib.convertFromOldFormat();

    calib.Kpwm-calib.k_pwm2i
    calib.Kbemf-calib.k_bemf
    calib.offset-calib.i_offset
end

save(calibrationMapFile,'calibrationMap');

clear calibrationMap;
end

% Create FullMotorTransFunc and copy from calibrationMap
load(calibrationMapFile,'calibrationMap');

calibrationMapFull = containers.Map();
for motorName = calibrationMap.keys
    calib = calibrationMap(motorName{1});
    calibrationMapFull(motorName{1}) = calib.copy();
end

