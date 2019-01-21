function plotTrainingData(obj,...
    dataPath,measedSensorList,measedPartsList,...
    model,taskSpecificParams)

% This function plots the acquired data for the friction or kcurr calibration
%   Detailed explanation goes here

% Get calibration map
calibrationMap = model.calibrationMap;

% Unwrap task specific parameters, defines:
% - frictionOrKcurr       -> = 'friction' for friction calibration
%                           = 'kcurr' for kcurr calibration
% - jointMotorCoupling   -> label for retrieving the currently calibrated
%                           joint/motor group info. Refer to jointsDbase
%                           class interface.
% - savePlot
% 
% - exportPlot
% 
% a joint/motor group info is formatted as follows:
% group.coupledJoints : ordered list of joint names (size 1 or n)
% group.coupledMotors : ordered list of MotorFriction object handles (same size)
% group.T             : 3x3 matrix or integer 1
% 
Init.unWrap(taskSpecificParams);

% Get the coupling info from the motor name. Each motor belongs to a single coupling set.
jointMotorCoupling = cell(model.jointsDbase.getJMcouplings('motors',{motorName})){1};

%% build input data for calibration
%
% build sensor data parser
dataLoadingParams = LowlevCurrCtrlCalibrator.buildDataLoadingParams(...
    model,measedSensorList,measedPartsList,...
    jointMotorCoupling.coupledJoints);

plot = false; loadJointPos = true;

data = SensorsData(dataPath,obj.subSamplingSize,...
    obj.timeStart,obj.timeStop,plot,...
    calibrationMap,obj.filtParams);
data.buildInputDataSet(loadJointPos,dataLoadingParams);

%============================================================
% Plot the data in order to control the measurements quality
% 

% define data folder
plotFolder = [dataPath '/diag'];

% Create figures handler
figuresHandler = DiagPlotFiguresHandler(plotFolder);
obj.figuresHandlerMap(obj.task) = figuresHandler;

% Joint encoder velocities and currents, and motor PWM measurements can be
% retrieved from the 'data' structure:
% 
% For N samples of dimension D (group of D coupled joints), we get:
% 
% Motor velocities table [DxN] : data.parsedParams.dqMRad_<label>
% joint PWM table [DxN]        : data.parsedParams.pwm_<label>
% joint currents table [DxN]    : data.parsedParams.curr_<label>
% 
% Parameter names finishing by 's' are the ones recomputed after resampling
% (refer to 'subSamplingSize' in lowLevCtrlCalibratorDevConfig.m config
% file).
% 
time = data.parsedParams.time;

% Get the calibrated joint index as mapped in the motors control board server.
% jointIdxes = model.jointsDbase.getAxesIdxesFromCtrlBoard('joints',jointMotorCoupling.coupledJoints);
[~,motorIdx] = ismember(motorName,jointMotorCoupling.coupledMotors);

% Get respective currents (matrix 6xNsamples)
currMotorG  = data.parsedParams.(['curr_' jointMotorCoupling.part '_state'])(motorIdx,:);
time = data.parsedParams.(['time_' jointMotorCoupling.part '_state']);

switch frictionOrKcurr
    case 'friction'
        dqMradG = data.parsedParams.(['dqMRad_' jointMotorCoupling.part '_state'])(motorIdx,:);
        
        % filtered Curr(time) & dqM(time)
        Plotter.plot2funcTimeseriesYY(...
            figuresHandler,...
            'Motor velocity and current over time','motorVel_N_CurrFilt',...
            time,dqMradG,time,currMotorG,...
            'Motor velocity (radians/s)','Motor current (A)');
        
        % filtered Curr VS dqM
        Plotter.plot2dDataNfittedModel(...
            figuresHandler,...
            'Motor velocity to current model','motorVel2curr',...
            dqMradG,currMotorG,...
            [],[],...
            'Motor velocity (radians/s)','Motor current (A)',...
            'Training data','');
        
    case 'kcurr'
        pwmDutyCycle = ...
            data.parsedParams.(['pwm_' jointMotorCoupling.part '_state'])(motorIdx,:);
        
        % filtered Curr(time) & pwm(time)
        Plotter.plot2funcTimeseriesYY(...
            figuresHandler,...
            'Motor PWM and current over time','motorPWM_N_currFilt',...
            time,pwmDutyCycle,time,currMotorG,...
            'Motor PWM (duty cycle)','Motor current (A)');
        
        % filtered Curr VS pwm
        Plotter.plot2dDataNfittedModel(...
            figuresHandler,...
            'Motor PWM to current model','motorPWM2curr',...
            pwmDutyCycle,currMotorG,...
            [],[],...
            'Motor PWM (duty cycle)','Motor current (A)',...
            'Training data','');
        
    otherwise
        error('plotTrainingData: unknown calibration type !!');
end

% Define callback for saving the plots into matlab figure files and
% eventually export them to PNG files.
obj.savePlotCallback = @() obj.savePlot(figuresHandler,savePlot,exportPlot,dataPath);

end

