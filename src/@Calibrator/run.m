function run(obj,init,model,lastAcqSensorDataAccessorMap)
% Calibrates the sensors using the data accessed through 'lastAcqSensorDataAccessorMap'

% Get or acquire sensor data
obj.getOrAcquireData(init,model,lastAcqSensorDataAccessorMap);

% Save eventual changes of last acquired data accessors to file
save('lastAcqSensorDataAccessorMap.mat','lastAcqSensorDataAccessorMap');

% Calibrate the joint encoders
obj.runCalibratorOrDiagnosis(init,model,@obj.calibrateSensors,obj.calibedSensorType);

% Run diagnosis on acquired data.
if init.runDiagnosis
    % Diagnosis function: doesn't require 'calibedParts' & 'calibedJointsIdxes'.
    diagFuncH = @(path,~,sensors,parts,model,taskSpec) ...
        SensorDiagnosis.runDiagnosis(...
        path,sensors,parts,model,taskSpec,... % actual params passed through the func handle
        obj.figuresHandlerMap,obj.task);              % params specific to this calibrator
    % Run diagnosis plotters for all acquired data, so for each acquired data accessor.
    obj.runCalibratorOrDiagnosis(init,model,diagFuncH,obj.calibedSensorType);
end

end
