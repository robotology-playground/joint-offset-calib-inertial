%% clear all variables and close all previous figures
clear
close all
clc

%% Main interface parameters ==============================================

% 'matFile' or 'dumpFile' mode
saveToCache = false;
loadFromCache = false;
saveFiltParams = false;
loadFiltParams = false;

% model and data capture file
dataPath  = '../../data/dumper/icubSim.accelerometersCalibrator#25.seq#25';
modelPath = '../models/icubSim/icub.urdf';
dataSetNb = '';
contextPath = '../../data/calibration/sgolayFiltParams.mat';

% Start and end point of data samples
timeStart = 1;  % starting time in capture data file (in seconds)
timeStop  = -1; % ending time in capture data file (in seconds). If -1, use 
                % the end time from log
% filtering/subsampling: the main single data bucket of (timeStop-timeStart)/10ms 
% samples is sub-sampled to 'subSamplingSize' samples for running the ellipsoid fitting.
subSamplingSize = 1000;
% Default filter params
filtParams.type = 'none';

% define the limb from which we will calibrate all the sensors.
% Activate all the sensors of that limb.
jointsToCalibrate.parts = {'right_leg'};

%%=========================================================================

%% set init parameters 'modelParams'
%
measedSensorList = {'acc','joint'};
measedPartsList = {{'right_leg'},{'right_leg'}};
mtbSensorAct.left_arm = [10:13 8:9 7];
mtbSensorAct.right_arm = [10:13 8:9 7];
mtbSensorAct.left_leg = 1:13;
mtbSensorAct.right_leg = 1:13;
mtbSensorAct.torso = 7:10;
mtbSensorAct.head = 1;

modelParams = model.buildModelParams(...
    measedSensorList,measedPartsList,...
    {},[],...  % no need for calibration parts information
    mtbSensorAct);

%% Get sensor output data
%
if loadFromCache
    loadSource = 'matFile';
else
    loadSource = 'dumpFile';
end

% Retrieve all captured data 
switch loadSource
    case 'cache'
        load([dataPath '/dataCache.mat'],'data');
        
    case 'dumpFile'
        % build sensor data parser
        plot_ = false; loadJointPos = false;
        data = SensorsData(dataPath,subSamplingSize,...
            timeStart,timeStop,plot_,[],filtParams);
        [sensorsIdxListFile,~] = data.buildInputDataSet(loadJointPos,modelParams);
        
        % Save data in a Matlab file for faster access in further runs
        if saveToCache
            save([dataPath '/dataCache.mat'],'data');
        end
        
    otherwise
        disp('Unknown data source !!')
end

acc_list = 1:length(sensorsIdxListFile);

% get time series of ys_xxx_acc [3xnSamples] from captured data,
sensMeasCell = cell(1,length(sensorsIdxListFile));
timeCell = cell(1,length(sensorsIdxListFile));

for acc_i = acc_list
    % raw sensor measurements components
    y = ['y_' data.labels{sensorsIdxListFile(acc_i)}];
    eval(['sensMeas = data.parsedParams.' y ';']);
    sensMeasCell{1,acc_i} = sensMeas'*data.calib{sensorsIdxListFile(acc_i)}.gain;
    % time
    t = ['time_' data.labels{sensorsIdxListFile(acc_i)}];
    eval(['time = data.parsedParams.' t ';']);
    timeCell{1,acc_i} = data.tInit + time';
end

%% Plot data, apply sgolay filter and tune its parameters
%

for acc_i = acc_list
    % init filter parameter and referenced object
    filterContext = FilterContext(5,601,timeCell{1,acc_i},sensMeasCell{1,acc_i},contextPath);
    
    % create figure and press-key handler
    figure('Name',['{x,y,z} Components of raw sensor ' num2str(sensorsIdxListFile(acc_i))],...
        'WindowKeyPressFcn',{@FilterContext.tuneFilter,filterContext});
    set(gcf,'PositionMode','manual','Units','normalized','outerposition',[0 0 1 1]);
    
    % Plot original signal components X, Y, Z
    ax = subplot(3,1,1);
    title('X component','Fontsize',16,'FontWeight','bold');
    grid ON;
    hold on;
    plot(timeCell{1,acc_i},sensMeasCell{1,acc_i}(:,1),'r-','lineWidth',2.0);
    xlabel('Time (sec)','Fontsize',12);
    ylabel('proper a_x (m/s^2)','Fontsize',12);
    hold off;
    
    ay = subplot(3,1,2);
    title('Y component','Fontsize',16,'FontWeight','bold');
    grid ON;
    hold on;
    plot(timeCell{1,acc_i},sensMeasCell{1,acc_i}(:,2),'r-','lineWidth',2.0);
    xlabel('Time (sec)','Fontsize',12);
    ylabel('proper a_y (m/s^2)','Fontsize',12);
    hold off;

    az = subplot(3,1,3);
    title('Z component','Fontsize',16,'FontWeight','bold');
    grid ON;
    hold on;
    plot(timeCell{1,acc_i},sensMeasCell{1,acc_i}(:,3),'r-','lineWidth',2.0);
    xlabel('Time (sec)','Fontsize',12);
    ylabel('proper a_z (m/s^2)','Fontsize',12);
    hold off;
    
    % register the subplot axis for allowing the callback function to plot
    % the filtered components
    filterContext.regSubPlots(ax,ay,az);
end

