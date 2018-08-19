classdef SensorMeasurementsEstimator < handle
    %This class computes the predicted sensors measurements
    %   using iDynTree estimator and all the sensors information from the
    %   URDF model.
    
    properties(Access=public)
        modelName;
        estimator;             % estimator of sensor measurements using Forward Kinematics
        grav_idyn;             % gravity iDynTree object
        dofs;                  % joint information: DOF
        q_idyn;                % joint position iDynTree object
        dq_idyn;               % joint velocity iDynTree object
        d2q_idyn;              % joint acceleration iDynTree object
        base_link_index;       % input param of estimator. iDynTree model indexing
        fullBodyUnknowns;      % input param of estimator
        estMeasurements;       % input param of estimator
        allMeas_idyn;          % temporary serialized estimated measurements
        sink1;                 % sink for output estContactForces
        sink2;                 % sink for output estJointTorques
        estimatedSensorLinAcc;     % predicted accelerometer measurement on sensor frame
        estimatedSensorAngVel;     % predicted gyroscope measurement on sensor frame
        fixedBasePose;             % full tree joint positions (including base link)
        nbFTs;
        nbAccs;
        nbGyros;
        nbThAxAngAccs;
        nbThAxFTs;
    end
    
    methods
        function obj = SensorMeasurementsEstimator(modelName,modelPath,varargin)
            % save model name
            obj.modelName = modelName;
            
            % Create estimator class
            obj.estimator = iDynTree.ExtWrenchesAndJointTorquesEstimator();
            
            % Load model and sensors from the URDF file
            if (numel(varargin)==0)
                obj.estimator.loadModelAndSensorsFromFile(modelPath);
            elseif (numel(varargin)==2)
                aModel = varargin{1};
                aSensorsList = varargin{2};
                obj.estimator.setModelAndSensors(aModel,aSensorsList);
            else
                error('Wrong number of input arguments!');
            end
            
            obj.nbFTs = obj.estimator.sensors.getNrOfSensors(iDynTree.SIX_AXIS_FORCE_TORQUE);
            obj.nbAccs = obj.estimator.sensors.getNrOfSensors(iDynTree.ACCELEROMETER);
            obj.nbGyros = obj.estimator.sensors.getNrOfSensors(iDynTree.GYROSCOPE);
            obj.nbThAxAngAccs = obj.estimator.sensors.getNrOfSensors(iDynTree.THREE_AXIS_ANGULAR_ACCELEROMETER);
            obj.nbThAxFTs = obj.estimator.sensors.getNrOfSensors(iDynTree.THREE_AXIS_FORCE_TORQUE_CONTACT);
            
            % Prepare inputs for updating the kinematics information in the estimator
            %
            % Compute the kinematics information necessary for the accelerometer sensor
            % predicted measurements computation. We assume the robot root link is fixed to
            % the ground. We then assume to know the gravity (ground truth) projected on the
            % frame (base_link) fixed to the root link. This is a default value that shall be
            % used if none is passed with the predictMeasurements() function call. 
            %
            obj.grav_idyn = iDynTree.Vector3();
            obj.grav_idyn.fromMatlab([0.0 0.0 -9.81]');
            
            % Base link index for later applying forward kynematics
            obj.base_link_index = obj.estimator.model.getFrameIndex('base_link');
            
            % Set the position of base link
            obj.fixedBasePose = iDynTree.FreeFloatingPos(obj.estimator.model);
            obj.fixedBasePose.worldBasePos().setRotation(iDynTree.Rotation.Identity());
            obj.fixedBasePose.worldBasePos().setPosition(iDynTree.Position.Zero());

            % Specify unknown wrenches (unknown Full wrench applied at the origin of the base_link frame)
            % The fullBodyUnknowns is a class storing all the unknown external wrenches acting on
            % the robot: we consider the pole/ground reaction on the base link as the only
            % external force.
            obj.fullBodyUnknowns = iDynTree.LinkUnknownWrenchContacts(obj.estimator.model);
            obj.fullBodyUnknowns.addNewUnknownFullWrenchInFrameOrigin(...
                obj.estimator.model, ...
                obj.base_link_index);
            
            % Print the unknowns to make sure that everything is properly working
            obj.fullBodyUnknowns.toString(obj.estimator.model())
            
            % The estimated sensor measurements:
            % `estimator.sensors()` gets used sensors (returns `SensorList`)
            % ex: `estimator.sensors.getNrOfSensors(iDynTree.ACCELEROMETER)`
            %     `estimator.sensors.getSensor(iDynTree.ACCELEROMETER,1)`
            obj.estMeasurements = iDynTree.SensorsMeasurements(obj.estimator.sensors);
            obj.allMeas_idyn = iDynTree.VectorDynSize();
            
            % Joint states and sensor measurements containers
            % Get joint information (DOF) and create joint position iDynTree objects 
            % Note: 'JointPosDoubleArray' is a special type for future evolution which
            % will handle quaternions. But for now the type has the format as
            % 'JointDOFsDoubleArray'.
            obj.dofs = obj.estimator.model.getNrOfDOFs();
            obj.q_idyn   = iDynTree.JointPosDoubleArray(obj.dofs);
            obj.dq_idyn  = iDynTree.JointDOFsDoubleArray(obj.dofs);
            obj.d2q_idyn = iDynTree.JointDOFsDoubleArray(obj.dofs);
            
            % estimation outputs
            obj.estimatedSensorLinAcc = iDynTree.LinearMotionVector3();
            obj.estimatedSensorAngVel = iDynTree.AngularMotionVector3();
            
            % Memory allocation for unused output variables
            obj.sink1 = iDynTree.LinkContactWrenches(obj.estimator.model);
            obj.sink2 = iDynTree.JointDOFsDoubleArray(obj.dofs);
        end
        
        % Returns the sensors list currently handled by the Predictor
        function sensorList = getSensorList(obj)
            sensorList = obj.estimator.sensors;
        end
        
        % Update the fixed base position and orientation
        function updateBasePose(obj,worldBasePose)
            %obj.fixedBasePose.worldBasePos() = worldBasePose;
        end
        
        % Add sensors to the list of sensors and update the estimated
        % measurements sink variable
        [sensorIdxes] = addSensors(obj,aSensorsList);
        
        % Set the fixed base index and update settings accordingly
        changeFixedBase(obj,linkName);
        
        % Computes the predicted sensor measurements
        [FTsMeas,AccsMeas,GyrosMeas,ThAxAngAccsMeas,ThAxFTsMeas] = getEstimatedMeasurements(obj,q,dq,d2q,gravity);
    end
    
    methods(Static)
        % Create sensors to be added to a desired link. Supported sensors:
        % ACCELEROMETER,GYROSCOPE,THREE_AXIS_ANGULAR_ACCELEROMETER.
        sensorList = createSensorList(model,sensorTypes,sensorNames,parentLinkNames,linkSensorHtransforms);
    end
end
