function calibrateSensors(obj,...
    dataPath,~,measedSensorList,measedPartsList,...
    model,taskSpecificParams)

% Get calibration map
calibrationMap = model.calibrationMap;

% Unwrap task specific parameters, defines:
% - frictionOrKtau       -> = 'friction' for friction calibration
%                           = 'ktau' for ktau calibration
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
dataLoadingParams = LowlevTauCtrlCalibrator.buildDataLoadingParams(...
    model,measedSensorList,measedPartsList,...
    jointMotorCoupling.coupledJoints);

plot = false; loadJointPos = true;
data = SensorsData(dataPath,obj.subSamplingSize,...
    obj.timeStart,obj.timeStop,plot,...
    calibrationMap,obj.filtParams);
data.buildInputDataSet(loadJointPos,dataLoadingParams);

%% Fitting process implementation.
% Joint encoder velocities and torques, motor PWM measurements can be
% retrieved from tha 'data' structure:
% 
% For N samples of dimension D (group of D coupled joints), we get:
% 
% joint velocities table [DxN] : data.parsedParams.dqMRad_<label>
% joint PWM table [DxN]        : data.parsedParams.pwm_<label>
% joint torques table [DxN]    : data.parsedParams.tau_<label>
% 
% Parameter names finishing by 's' are the ones recomputed after resampling
% (refer to 'subSamplingSize' in lowLevCtrlCalibratorDevConfig.m config
% file).
% 

% Get the calibrated joint index as mapped in the motors control board server.
%jointIdxes = model.jointsDbase.getAxesIdxesFromCtrlBoard('joints',jointMotorCoupling.coupledJoints);
[~,motorIdx] = ismember(motorName,jointMotorCoupling.coupledMotors);

% Get respective torques (matrix 6xNsamples)
tauJoints  = data.parsedParams.(['tau_' jointMotorCoupling.part '_state']);

% Get respective currents (matrix 6xNsamples)
currMotorG  = data.parsedParams.(['curr_' jointMotorCoupling.part '_state'])(motorIdx,:);

% FRICTION parameters
%
% We express the joint velocities and torques w.r.t. the motor respective
% quantities using the coupling matrix Tm2j (motor to joint) and gearbox ratios:
% 
% dq_j = Tm2j * Gm2j * dq_m
%
% Where Gm2j is a diagonal matrix. We then pose the conservation of
% transmission power:
%
% dq_j' * Tau_j = dq_m' * Tau_m
%
% <=> dq_m' * Gm2j' * Tm2j' * Tau_j = dq_m' * Tau_m  ∀dq_m
%
% <=> Tau_m = Gm2j' * Tm2j' * Tau_j
% 
% Anyway we consider here the motor and gearbox as a single block, and
% the velocity and torque as the outputs of that same block:
%
% xVar = S * Gm2j * dq_m
% yVar = S * Tm2j' * Tau_j
%
% Where Gm2j is a diagonal matrix whose diagonal terms are represented by
% gearboxDqM2Jratios, and S is a selective matrix. So, for motorIdx "i",
% S=[0..0 1 0..0] (ith column set to 1). For any matrix A, we get S * A =
% A(i,:), and A * S' = A(:,i). We get or each sample at instant "t":
%
% xVar = Gm2j(i,:) * dq_m = gearboxDqM2Jratios(i) * dq_m(i)
% yVar = (Tm2j * S')' * Tau_j = Tm2j(:,i)' * Tau_j
%
tauMotorG = jointMotorCoupling.Tm2j(:,motorIdx)' * tauJoints;

switch frictionOrKtau
    case 'friction'
        % get motor velocity * Gm2j (rad/s) to be the x axis variable
        xVar_tau = ...
            jointMotorCoupling.gearboxDqM2Jratios{motorIdx} ...
            * data.parsedParams.(['dqMRad_' jointMotorCoupling.part '_state'])(motorIdx,:);
        xVar_curr = data.parsedParams.(['dqMRad_' jointMotorCoupling.part '_state'])(motorIdx,:);
        
    case 'ktau'
        % get motor PWM (% Fullscale) to be the x axis variable
        xVar_tau = data.parsedParams.(['pwm_' jointMotorCoupling.part '_state'])(motorIdx,:);
        xVar_curr = xVar_tau;
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
[thetaPosXvar,thetaNegXvar] = Regressors.normalEquationAsym(xVar_tau',tauMotorG');

%% Convert theta vector to model parameters (motor calibration) and save it to the calibration map

% Get the motor calibration or create a new one. The method returns a
% handle on the MotorTransFunc object.
calib = FullMotorTransFunc.GetMotorTransFunc(motorName,calibrationMap);

switch frictionOrKtau
    case 'friction'
        % TAU CTRL
        % Check that the model is symmetrical
        [KcP, KcN, KvP, KvN] = deal(thetaPosXvar(1),thetaNegXvar(1),thetaPosXvar(2),thetaNegXvar(2));
        if abs(KcP+KcN)>abs(KcP)/100 || abs(KvP-KvN)>abs(KvP)/100
            warning('calibrateSensors: The friction model is not symmetrical');
        end
        % Run a fitting again but matching a single Kc and a single Kv
        % For non-coupled joints fit also the static friction parameter
        if(jointMotorCoupling.Tm2j == 1)
            fittedModelTau = Regressors.frictionModel2(xVar_tau',tauMotorG');
            calib.setFriction(fittedModelTau.theta(1), fittedModelTau.theta(2));
            calib.setStiction(fittedModelTau.theta(3),fittedModelTau.theta(4));
        else
            
            fittedModelTau = Regressors.frictionModel2(xVar_tau',tauMotorG');
            calib.setFriction(fittedModelTau.theta(1), fittedModelTau.theta(2));
            calib.setStiction(nan,nan);
        end
        
        % CURR CTRL
        fittedModelCurr = Regressors.normalEquation(xVar_curr',currMotorG');
        if abs(fittedModelCurr.theta(1))>1e-3 % Tau offset
            warning('calibrateSensors: There is a current offset in the model motor velocity to current !!');
        end
        calib.setKbemf(fittedModelCurr.theta(2));
        
    case 'ktau'
        % TAU CTRL
        % Check that the model is symmetrical
        [KoffP, KoffN, KpwmP, KpwmN] = deal(thetaPosXvar(1),thetaNegXvar(1),thetaPosXvar(2),thetaNegXvar(2));
        if ...
                abs(KoffP+KoffN)>abs(KoffP)/100 ...
                || abs(KpwmP-KpwmN)>abs(KpwmP)/100
            warning('calibrateSensors: The Ktau model is not symmetrical');
        end
        % Run a fitting again but matching a single Ktau
        fittedModelTau = Regressors.pwmModel1Sym(xVar_tau',tauMotorG');
        if abs(fittedModelTau.theta(1))>1e-3 % Tau offset
            warning('calibrateSensors: There is a torque offset in the model PWM to torque !!');
        end
        calib.setKpwm(fittedModelTau.theta(2));
        
        % CURR CTRL
        % Run a fitting again but matching a single Kcurr
        fittedModelCurr = Regressors.normalEquation(xVar_curr',currMotorG');
        calib.setIoffset(fittedModelCurr.theta(1));
        calib.setKpwm2i(fittedModelCurr.theta(2));
        
        % CURR TO TORQUE CTRL
        calib.computeKt();
        calib.computeKvmech(jointMotorCoupling.gearboxDqM2Jratios{motorIdx});
        
        disp(calib);
        
    otherwise
        error('calibrateSensors: unknown calibration type !!');
end

% Plot fitted model over acquired data.
obj.plotModel(frictionOrKtau,fittedModelTau,xVar_tau,1000);
obj.plotModel(frictionOrKtau,fittedModelCurr,xVar_curr,1000);

end
