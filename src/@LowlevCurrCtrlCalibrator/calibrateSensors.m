function calibrateSensors(obj,...
    dataPath,~,measedSensorList,measedPartsList,...
    model,taskSpecificParams)

% Get calibration map
calibrationMap = model.calibrationMap;

% Unwrap task specific parameters, defines:
% - frictionOrKcurr       -> = 'friction' for friction calibration
%                           = 'kcurr' for kcurr calibration
% - jointMotorCoupling   -> label for retrieving the currently calibrated
%                           joint/motor group info. Refer to jointsDbase
%                           class interface.
% 
% a joint/motor group info is formatted as follows:
% group.coupledJoints : ordered list of joint names (size 1 or n)
% group.coupledMotors : ordered list of MotorFriction object handles (same size)
% group.T             : 3x3 matrix or integer 1
% 
Init.unWrap(taskSpecificParams);

% Get the coupling info from the motor name. Each motor belongs to a single coupling set
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

%% Fitting process implementation.
% Joint encoder velocities and currents, motor PWM measurements can be
% retrieved from tha 'data' structure:
% 
% For N samples of dimension D (group of D coupled joints), we get:
% 
% joint velocities table [DxN] : data.parsedParams.dqMRad_<label>
% joint PWM table [DxN]        : data.parsedParams.pwm_<label>
% joint currents table [DxN]    : data.parsedParams.curr_<label>
% 
% Parameter names finishing by 's' are the ones recomputed after resampling
% (refer to 'subSamplingSize' in lowLevCtrlCalibratorDevConfig.m config
% file).
% 

% Get the calibrated joint index as mapped in the motors control board server.
%jointIdxes = model.jointsDbase.getAxesIdxesFromCtrlBoard('joints',jointMotorCoupling.coupledJoints);
[~,motorIdx] = ismember(motorName,jointMotorCoupling.coupledMotors);

% Get respective currents (matrix 6xNsamples)
currMotorG  = data.parsedParams.(['curr_' jointMotorCoupling.part '_state']);

switch frictionOrKcurr
    case 'friction'
        % get motor velocity * Gm2j (rad/s) to be the x axis variable
        xVar = data.parsedParams.(['dqMRad_' jointMotorCoupling.part '_state'])(motorIdx,:);
        
    case 'kcurr'
        % get motor PWM (% Fullscale) to be the x axis variable
        xVar = data.parsedParams.(['pwm_' jointMotorCoupling.part '_state'])(motorIdx,:);
    otherwise
        error('calibrateSensors: unknown calibration type !!');
end

% Fit the model using linear regression in the closed form
% (pseudo-inverse). The regression result is returned in 'thetaPosXvar' and
% 'thetaNegXvar' following the mapping:
% -> thetaPosXvar(1) = fitting model's pos. offset
% -> thetaNegXvar(1) = fitting model's neg. offset
% -> thetaPosXvar(2) = fitting model's pos. slope
% -> thetaNegXvar(2) = fitting model's neg. slope
% 
[thetaPosXvar,thetaNegXvar] = Regressors.normalEquationAsym(xVar',currMotorG');

%% Convert theta vector to model parameters (motor calibration) and save it to the calibration map

% Get the motor calibration or create a new one. The method returns a
% handle on the MotorTransFunc object.
calib = ElectricalMotorTransFunc.GetMotorTransFunc(motorName,calibrationMap);

switch frictionOrKcurr
    case 'friction'
        % Check that the model is symmetrical
        [KcP, KcN, KvP, KvN] = deal(thetaPosXvar(1),thetaNegXvar(1),thetaPosXvar(2),thetaNegXvar(2));
        if abs(KcP+KcN)>abs(KcP)/100 || abs(KvP-KvN)>abs(KvP)/100
            warning('calibrateSensors: The friction model is not symmetrical');
        end
        % Run a fitting again but matching a single Kv and removing the
        % offset
        fittedModel = Regressors.normalEquation(xVar',currMotorG'-calib.i_offset);
        if abs(fittedModel.theta(1))>1e-3 % Tau offset
            warning('calibrateSensors: There is a current offset in the model motor velocity to current !!');
        end
        calib.setKbemf(fittedModel.theta(2));
   
    case 'kcurr'
        % Check that the model is symmetrical
        [KoffP, KoffN, KpwmP, KpwmN] = deal(thetaPosXvar(1),thetaNegXvar(1),thetaPosXvar(2),thetaNegXvar(2));
        if ...
                abs(KoffP+KoffN)>abs(KoffP)/100 ...
                || abs(KpwmP-KpwmN)>abs(KpwmP)/100
            warning('calibrateSensors: The Kcurr model is not symmetrical');
        end
        % Run a fitting again but matching a single Kcurr
        fittedModel = Regressors.normalEquation(xVar',currMotorG');
        calib.setIoffset(fittedModel.theta(1));
        calib.setKpwm2i(fittedModel.theta(2));
        
    otherwise
        error('calibrateSensors: unknown calibration type !!');
end

% Plot fitted model over acquired data.
obj.plotModel(frictionOrKcurr,fittedModel,xVar,1000);

end
