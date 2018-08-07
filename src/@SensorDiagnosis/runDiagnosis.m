function runDiagnosis(...
    dataPath,measedSensorList,measedPartsList,model,taskSpecificParams,...
    figuresHandlerMap,task) % params specific to this diagnosis function

% Parameters specific to this diagnosis function:
% [in]     model: robot model (calibration data and model dynamics estimator)
% [in]     calibrationMap: sensors calibration parameters
% [in/out] figuresHandlerMap: Map with class objects handling the figures generated
% in this function.
% [in]     task: calibration task that generated the sensor data.

% Get calibration map
calibrationMap = model.calibrationMap;

% Unwrap task specific parameters (defines 'calibedJointsIdxes')
Init.unWrap(taskSpecificParams);

% Advanced interface parameters
run sensorDiagnosisDevConfig;

%% set init parameters 'modelParams'
%
modelParams = model.buildModelParams(...
    measedSensorList,measedPartsList,...
    {},[],...  % no need for calibration parts information
    mtbSensorAct);

%% Update iterator and prepare log folders/files
%

% define data folder
dataFolder = [dataPath '/diag'];

% Create figures handler
figuresHandler = DiagPlotFiguresHandler(dataFolder);
figuresHandlerMap(task) = figuresHandler;

%% ===================================== CALIBRATION VALIDATION ==============================
%

%% build input data before and after calibration
%
switch loadSource
    case 'cache'
        load([dataPath '/dataCache.mat']);
    case 'dumpFile'
        % Build input data without calibration applied
        plot = false;
        data.bc = SensorsData(dataPath,subSamplingSize,...
            timeStart,timeStop,plot);
        [~,sensMeasCell.bc] = data.bc.buildInputDataSet(loadJointPos,modelParams);
        
        % Build input data with calibration applied
        data.ac = SensorsData(dataPath,subSamplingSize,...
            timeStart,timeStop,plot,calibrationMap);
        [sensorsIdxListFile,sensMeasCell.ac] = data.ac.buildInputDataSet(loadJointPos,modelParams);
        
        % Save data in a Matlab file for faster access in further runs
        if saveToCache
            save([dataPath '/dataCache.mat'],'data','sensorsIdxListFile','sensMeasCell');
        end
    otherwise
        disp('Unknown data source !!')
end

%% Check anysotropy of gains and offsets
%
SensorDiagnosis.checkAccelerometersAnysotropy(...
    data.bc,data.ac,sensorsIdxListFile,...
    sensMeasCell.bc,sensMeasCell.ac,...
    figuresHandler);

if loadJointPos
    %% Generate predicted measurements
    %
    % create the calibration context implementing the cost function
    myCalibContext = CalibrationContextBuilder(model.estimator);
    costFunction = @myCalibContext.costFunctionSigma;
    nrOfMTBAccs = length(sensorsIdxListFile);

    % init joints and sensors lists
    myCalibContext.buildSensorsNjointsIDynTreeListsForActivePart(data.bc,modelParams);
    
    % load joint positions
    myCalibContext.loadJointNsensorsDataSubset(1:data.bc.nSamples);
    
    % cost before calibration
    [initialCost,sensMeasCell.bc,sensEstCell.bc] = costFunction(0,data.bc,1:data.bc.nSamples,@lsqnonlin,false,'');
    fprintf('Mean cost before calibration (in (m.s^{-2})^2):\n %f\n',...
        (initialCost'*initialCost)/(nrOfMTBAccs*data.bc.nSamples));
    
    % cost after calibration
    [finalCost,sensMeasCell.ac,sensEstCell.ac] = costFunction(0,data.ac,1:data.ac.nSamples,@lsqnonlin,false,'');
    fprintf('Mean cost after calibration (in (m.s^{-2})^2):\n %f\n',...
        (finalCost'*finalCost)/(nrOfMTBAccs*data.ac.nSamples));
    
    %% Plot joint trajectories
    %
    SensorDiagnosis.plotJointTrajectories(data,modelParams,figuresHandler);
    
    %% Plot predicted sensor variables VS sensor measurements
    %
    angleList.bc = SensorDiagnosis.checkSensorMeasVsEst(...
        data,sensorsIdxListFile,...
        sensMeasCell.bc,sensEstCell.bc,...
        figuresHandler,'bc');
    angleList.ac = SensorDiagnosis.checkSensorMeasVsEst(...
        data,sensorsIdxListFile,...
        sensMeasCell.ac,sensEstCell.ac,...
        figuresHandler,'ac');
    
    %% Plot improvement of error angle between the the predicted sensor variables and
    %  the sensor measurements
    SensorDiagnosis.checkSensorMeasVsEstAngleImprovmt(...
        data,sensorsIdxListFile,...
        angleList.bc,angleList.ac,...
        figuresHandler);
end

% Save the plots into matlab figure files and eventually export them to PNG
% files.
if savePlot
    % save plots
    [figsFolder,iterator] = figuresHandler.saveFigures(exportPlot);
    % create log info file
    fileID = fopen([figsFolder '.txt'],'w');
    fprintf(fileID,'modelPath = %s\n',model.urdfModelFile);
    fprintf(fileID,'dataPath = %s\n',dataPath);
    fprintf(fileID,'calibration map :\n');
    fprintf(fileID,'\t sensors = %s\n',UI.cellArrayOfStr2str(', ',calibrationMap.keys));
%    fprintf(fileID,'\t values = %f\n',UI.cellArrayOfStr2str(', ',calibrationMap.values));
    fprintf(fileID,'iterator = %d\n',iterator);
    fclose(fileID);
end

end
