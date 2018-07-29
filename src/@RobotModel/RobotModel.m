classdef RobotModel < handle
    %RobotModel This class holds all the properties of the model
    %   The class constructor retrieves the model parameters from the URDF
    %   file: joints list, links list, sensor frames and types, DOFs of
    %   each joint. For that purpose, an iDynTree model 'iDynTree::Model'
    %   is created and populated by the main app script through the
    %   iDynTree bindings.
    %   It then associates the sensor frames to labels as an abstraction
    %   between the model specific frame names and the names used in the
    %   application interface.
    %   It then creates the associations 'part/limb' <--> (sensor frames,
    %   joints list, part DOF, ...)
    %   For the accelerometers calibration, we need to retrieve the sensor
    %   frame names from the URDF model for a later association to the
    %   'iDynTree.ExtWrenchesAndJointTorquesEstimator' for computing the
    %   predicted link accelerations.
    %   The model also holds the sensor calibration parameters and the
    %   joint/motor parameters (PWM to torque rate, friction parameters, ...).
    
    properties(Constant = true, Access = public)
        isOnline@System.Buffer = System.Buffer(true);
    end
    
    properties(GetAccess = public, SetAccess = protected)
        robotEnvNames@struct;
        urdfModelFile@char;
        link2partMapping@struct;
        calibrationMapFile@char;
        calibrationMap@containers.Map;
        estimator@iDynTree.ExtWrenchesAndJointTorquesEstimator;
        jointsDbase@JointsDbase;  % Joint database built from iDynTree model parameters
        sensorsDbase@SensorsDbase; % Sensor database built from iDynTree model parameters
    end
    
    methods(Access = public)
        % Constructor
        function obj = RobotModel(modelName,urdfModelFile,calibrationMapFile)
            % Refresh model configuration files
            obj.refreshModelConfig(modelName);
            
            % Set robot environment names from the model name (yarp port prefix, yarp
            % robot name)
            obj.robotEnvNames = obj.getRobotEnvNames(modelName);

            % Set parameters from environment if needed
            if isempty(urdfModelFile)
                % Let Yarp resource finder get the model path for the respective Yarp
                % robot name. Trash the error/warning output and get only the result path
                setenv('YARP_ROBOT_NAME',obj.robotEnvNames.yarpRobotName);
                [status,path] = system('yarp resource --find model.urdf 2> /dev/null');
                if status
                    error('robot model not found !!');
                end
                path = strip(path); % remove spaces from the sides of the string
                urdfModelFile = strip(path,'"'); % remove the quotation marks
            end
            if isempty(calibrationMapFile)
                calibrationMapFile = ['../../data/calibration/' modelName '_calibrationMap.mat'];
            end
            
            % set remaining class attributes
            obj.urdfModelFile = urdfModelFile;
            obj.calibrationMapFile = calibrationMapFile;
            
            % Load existing sensors calibration (joint encoders, inertial & FT sensors, etc)
            obj.loadCalibFromFile();
            
            %% Create the iDynTree estimator and model...
            %
            % Create an estimator class, load the respective model from URDF file
            
            % Create estimator class
            obj.estimator = iDynTree.ExtWrenchesAndJointTorquesEstimator();
            
            % Load model and sensors from the URDF file
            obj.estimator.loadModelAndSensorsFromFile(urdfModelFile);
            
            % Build a typical database from the URDF model parameters
            % previously loaded in iDynTree, allowing to query elements
            % matching specified properties.
            obj.jointsDbase = JointsDbase(obj.estimator.model,@obj.link2part,obj.robotEnvNames.yarpPortPrefix);
            obj.sensorsDbase = SensorsDbase(obj.estimator.sensors,@obj.link2part);
        end
        
        % Load all sensors calibration parameters from the file path set at
        % model object instantiation.
        loadCalibFromFile(obj);
        
        % Save all sensors calibration parameters to the file path set at
        % model object instantiation.
        saveCalibToFile(obj);
        
        % Refined display of the object
        display(obj);
        
        % Build old interface structure such that RobotModel class
        % introduction doesn't impact the calibrators and the yarp data
        % parsing current design.
        modelParams = buildModelParams(obj,...
            measedSensorList,measedPartsList,...
            calibedParts,calibedJointsIdxes,...
            mtbSensorAct);
        
        % Rebuild the 'jointsDbase' and 'sensorsDbase' databases.
        buildDatabase(obj);
        
        % Get the part the link is attached to
        part = link2part(obj,link);
    end
    
    methods(Access=protected)
        % Update Matlab path with model path and reload model configuration
        % data from the model folder
        refreshModelConfig(obj,modelName);
    end
    
    methods(Static = true, Access = public)
        robotEnvNames = getRobotEnvNames(modelName);
    end
end

