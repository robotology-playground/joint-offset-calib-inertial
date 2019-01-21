%% clear all variables and close all previous figures
clear
close all
clc

%Clear static data
clear classes;
System.clearTimers();

% Create YARP Network device, for initializing YARP classes for communication
yarp.Network.init();

% import constants
import System.Const;

% load application main interface parameters
init = Init.load('sensorSelfCalibratorInit');
