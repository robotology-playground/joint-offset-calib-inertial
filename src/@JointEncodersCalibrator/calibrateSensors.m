function calibrateSensors(~,...
    dataPath,calibedParts,measedSensorList,measedPartsList,...
    model,taskSpecificParams)

% Get calibration map
calibrationMap = model.calibrationMap;

% Unwrap task specific parameters (defines 'calibedJointsIdxes')
Init.unWrap(taskSpecificParams);

% Convert 'calibedJointsIdxes' (just unwrapped) to matlab indexes
calibedJointsIdxes = structfun(...
    @(field) field+1,calibedJointsIdxes,'UniformOutput',false);

% Advanced interface parameters
run jointEncodersCalibratorDevConfig;

% Set the init vector Dq0 for the optimisation
model.jointsDbase.setAllJointsMaxCalibDq0(calibedJointsMaxDq0);

[optimFunction,options] = JointEncodersCalibrator.getOptimConfig();

% Build joint encoders and inertial sensors parameters
modelParams = model.buildModelParams(...
    measedSensorList,measedPartsList,...
    calibedParts,calibedJointsIdxes,...
    mtbSensorAct);

% in target mode, don't apply any prior offsets
if strcmp(loadSource,'dumpFile')
    offsetsGridRange = 0;
    offsetedQsIdxs = 1;
end

% create the calibration context implementing the cost function
myCalibContext = CalibrationContextBuilder(model.estimator);
% % DEBUG
% waitforbuttonpress;
% list_kHsens = myCalibContext.getListTransforms('base_link');
% importFrames;
% 
% list_kHsens_left_leg = list_kHsens(3+[1:7 9:14],1);
% list_kHsens_left_leg_idx = 3+[1:7 9:14]-1;
% 
% for iterList = 1:13
%     myCalibContext.estimator.sensors.getAccelerometerSensor(list_kHsens_left_leg_idx(iterList)).getName
%     sum(sum(abs(list_kHsens_fromCREO{iterList}-list_kHsens_left_leg{iterList})))
% end

%%

% Cost Function used to optimise the offsets
eval(['costFunction = @myCalibContext.' costFunctionSelect]);


%% build input data for calibration
%

switch loadSource
    case 'cache'
        load([dataPath '/dataCache.mat'],'data');
        
    case 'dumpFile'
        % build sensor data parser
        plot = false; loadJointPos = true;
        data = SensorsData(dataPath,subSamplingSize,...
            timeStart,timeStop,plot,calibrationMap);
        data.buildInputDataSet(loadJointPos,modelParams);
        
        % Save data in a Matlab file for faster access in further runs
        if saveToCache
            save([dataPath '/dataCache.mat'],'data');
        end
        
    otherwise
        disp('Unknown data source !!')
end

%% init joints and sensors lists. The order in modelParams.jointMeasedParts sets 
%  the order in which the joints lists for all parts are concatenated, as well as
%  Dq0 and Dq.
Dq0 = myCalibContext.buildSensorsNjointsIDynTreeListsForActivePart(data,modelParams);


%% OPTIMIZATION
%

% Note: below init variables are considered independent from the offsets
%
% selecting a subset of samples (time series vector)
subsetVec_size = round(data.nSamples*subsetVec_size_frac);
subsetVec_idx = round(linspace(1,data.nSamples,subsetVec_size));
% Starting point for optimization and boundaries. The format of Dq is
% defined by Dq0. Dq0 
lowerBoundary = Dq0 - startPoint2Boundary;
upperBoundary = Dq0 + startPoint2Boundary;

% Build the offsets grid
offsetsConfigGrid = nDimGrid(length(offsetedQsIdxs), ...
                             offsetsGridRange, ...
                             offsetsGridResolution)

optimalDq = zeros(length(Dq0),number_of_subset_init,offsetsConfigGrid.nbVectors);
resnorm = zeros(1,number_of_subset_init,offsetsConfigGrid.nbVectors);
exitflag = zeros(1,number_of_subset_init,offsetsConfigGrid.nbVectors);
output = cell(1,number_of_subset_init,offsetsConfigGrid.nbVectors);

% pre-computed optimal joint offsets (TO BE REMOVED)
averageOptimalDq = 0;

% iterate over the joints offsets grid values
for offsetsConfigIdx = 1:offsetsConfigGrid.nbVectors
    
    % set the offsets from grid
    myCalibContext.DqiEnc(offsetedQsIdxs) = offsetsConfigGrid.getVector(offsetsConfigIdx);
    
    % run minimisation for every random subset of data.
    % 1 subset <=> all measurements for a given timestamp <=>1 column index of
    % table `q_xxx`, `dq_xxx`, `ddq_xxx`, `y_xxx_acc`, ...
    %
    % Define a random subset: X % of the total set of instants
    % We first shuffle the data. Then, at each loop iteration i,
    % we select the samples i to i+n, where n = subsetVec_size.
    
    % Load existing indexes permutation
    if loadRandomDataIdxes
        if exist(randomDataIdxesFile,'file') == 2
            load(randomDataIdxesFile,'subsetVec');
        end
        
        if ~exist('subsetVec','var')
            error('subsetVec not found');
        end
    else
        subsetVec = randperm(data.nSamples);
    end
    
    if saveRandomDataIdxes
        save(randomDataIdxesFile,'subsetVec');
    end
    
    %%
    for i = 1 : number_of_subset_init
        % select the samples i to i+n
        idxOffset = (i-1)*subsetVec_size;
        if shuffle
            subsetVec_idx = subsetVec(idxOffset+1:min(idxOffset+subsetVec_size,data.nSamples));
        else
            subsetVec_idx = idxOffset+1:min(idxOffset+subsetVec_size,data.nSamples);
        end
        
        % load joint positions
        myCalibContext.loadJointNsensorsDataSubset(subsetVec_idx);
        
        % cost before optimisation
        initialCost = costFunction(zeros(size(Dq0)),data,subsetVec_idx,@lsqnonlin,false,'');
        fprintf('Mean cost before optimization (in (m.s^{-2})^2):\n');
        (initialCost'*initialCost)/(data.nrOfMTBAccs*length(subsetVec_idx))
        
        % optimize
        %
        % Important note:
        % - Dq0 is the init vector for the optimization
        % - Dq is the main optimization variable (format set by Dq0)
        %
        funcProps = functions(optimFunction);
        funcName = funcProps.function;
        switch funcName
            case 'fminunc'
                [optimalDq(:,i,offsetsConfigIdx),  resnorm(1,i,offsetsConfigIdx), ...
                    exitflag(1,i,offsetsConfigIdx), output{1,i,offsetsConfigIdx}] ...
                    = optimFunction(@(Dq) costFunction(Dq,data,subsetVec_idx,optimFunction,false,''), ...
                    Dq0, options);
            case 'lsqnonlin'
                [optimalDq(:,i,offsetsConfigIdx), resnorm(1,i,offsetsConfigIdx), ~, ...
                    exitflag(1,i,offsetsConfigIdx), output{1,i,offsetsConfigIdx}, ~] ...
                    = optimFunction(@(Dq) costFunction(Dq,data,subsetVec_idx,optimFunction,false,''), ...
                    Dq0, [], [], options);
            otherwise
                % We are not computing optimalDq, but just using a previous
                % result for a performance evaluation
                optimalDq(:,i,offsetsConfigIdx) = averageOptimalDq;
        end
        optimalDq(:,i,offsetsConfigIdx) = mod(optimalDq(:,i,offsetsConfigIdx)+pi, 2*pi)-pi;
        % computed Dq and known a priori offset (offsetsConfigGrid.getVector(offsetsConfigIdx))
        % added to ground truth q in simulation, are opposite. Add them
        % and check the result is null.
        optimalDq(:,i,offsetsConfigIdx) = optimalDq(:,i,offsetsConfigIdx) + myCalibContext.DqiEnc;
        
        % cost after optimisation
        optimCost = costFunction(optimalDq(:,i,offsetsConfigIdx),data,subsetVec_idx,@lsqnonlin,false,'Optim');
        fprintf('Mean cost after optimization (in (m.s^{-2})^2):\n');
        (optimCost'*optimCost)/(data.nrOfMTBAccs*length(subsetVec_idx))
    end
end

% convert to degrees
optimalDq = optimalDq*180/pi
averageOptimalDq = mean(optimalDq,2);
% Standard deviation across offsets grid
std_optDq_offsetsGrid = std(optimalDq,0,3);
% Standard deviation across random subsets
std_optDq_subsets = std(optimalDq,0,2);


% Display the results
fprintf('Final optimization results. Each column stands for a random init of the data subset.\n');
fprintf('Optimal offsets Dq (in degrees):\n');
optimalDq
fprintf('Mean cost (in (m.s^{-2})^2):\n');
resnorm/(data.nrOfMTBAccs*length(subsetVec_idx))
fprintf('optimization function exit flag:\n');
exitflag
fprintf('other optimization info:\n');
output
fprintf('Mean optimal offsets Dq (in degrees):\n');
averageOptimalDq
fprintf('Standard deviation across offsets grid:\n');
std_optDq_offsetsGrid
fprintf('Standard deviation across random subsets:\n');
std_optDq_subsets

% Plot the results
JointEncodersCalibrator.plotJointsOffsets(averageOptimalDq,std_optDq_subsets);

%% Format and save calibration in the main calibration map
%

% Merge new calibrated joint offsets with old 'calibrationMap'.
% The result matrix optimalDq has the same format as Dq and Dq0.
% Dq0 results from the concatenation of the modelParams.jointsToCalibrate.calibedJointsDq0
% matrices.
[~,calibedPartsIdxes] = ismember(modelParams.calibedParts,modelParams.jointMeasedParts);

% Split computed offsets matrix into part wise cells
calib = mat2cell(averageOptimalDq,lengths(modelParams.jointsToCalibrate.calibedJointsDq0{calibedPartsIdxes}));

for iter = calibedPartsIdxes
    mapKey = strcat('jointsOffsets_',modelParams.jointMeasedParts{iter}); % set map key
    % get current value or set a default one (zeros)
    if isKey(calibrationMap,mapKey)
        mapValue = calibrationMap(mapKey); % get current value
    else
        mapValue = zeros(modelParams.jointsToCalibrate.jointsDofs{iter},1); % init default value
    end
    mapValue(modelParams.jointsToCalibrate.calibedJointsIdxes{iter}) = ...
        mapValue(modelParams.jointsToCalibrate.calibedJointsIdxes{iter}) + calib{iter}; % add calibrated values
    calibrationMap(mapKey) = mapValue; % add or overwrite element in the map
end

end
