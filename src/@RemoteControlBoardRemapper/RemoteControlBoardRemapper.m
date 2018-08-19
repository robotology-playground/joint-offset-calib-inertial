classdef RemoteControlBoardRemapper < handle
    % This class creates a YARP network device and a remote controlboard
    % remapper device for controlling any desired joint through the YARP
    % ports.
    %
    % robotname : [string] 'icub' or 'icubSim'
    % jointsList: [list of strings] for instance {'l_hip_pitch','l_hip_roll','l_knee'}
    %
    
    properties(Constant)
        defaultSpeed = 10; % m/s
        defaultAcc   = 2;  % m/s^2
        
        ctrlModeVocabDef = {...
            'idlectrl'  , y.VOCAB_CM_IDLE     ;
            'torqctrl'  , y.VOCAB_CM_TORQUE   ;
            'ctrl'      , y.VOCAB_CM_POSITION ;
            'ctrldir'   , y.VOCAB_CM_POSITION_DIRECT ;
            'velctrl'   , y.VOCAB_CM_VELOCITY ;
            'curctrl'   , y.VOCAB_CM_CURRENT  ;
            'pwmctrl'   , y.VOCAB_CM_PWM
            };
        
        ctrlMode2vocab = containers.Map(...
            RemoteControlBoardRemapper.ctrlModeVocabDef(:,1),...
            RemoteControlBoardRemapper.ctrlModeVocabDef(:,2));
        
        vocab2ctrlMode = containers.Map(...
            RemoteControlBoardRemapper.ctrlModeVocabDef(:,2),...
            RemoteControlBoardRemapper.ctrlModeVocabDef(:,1));
        
        pidTypeVocabDef = {...
            'posPID' , y.VOCAB_PIDTYPE_POSITION;
            'velPID' , y.VOCAB_PIDTYPE_VELOCITY;
            'torqPID', y.VOCAB_PIDTYPE_TORQUE  ;
            'currPID', y.VOCAB_PIDTYPE_CURRENT
            };
        
        pidType2vocab = containers.Map(...
            RemoteControlBoardRemapper.pidTypeVocabDef(:,1),...
            RemoteControlBoardRemapper.pidTypeVocabDef(:,2));
        
        vocab2pidType = containers.Map(...
            RemoteControlBoardRemapper.pidTypeVocabDef(:,1),...
            RemoteControlBoardRemapper.pidTypeVocabDef(:,2));
    end
    
    properties(SetAccess = protected, GetAccess = public)
        robotModel;
        jointsList={};
        motorsList={};
        % YARP objects
        axesNames; axesList; remoteControlBoards; remoteControlBoardsList;
        options;
        driver@yarp.PolyDriver;
        iencs@yarp.IEncoders;          % IEncoders interface for reading joint encoders position and velocity
        imotorencs@yarp.IMotorEncoders % IMotorEncoders interface for reading motor encoders position and velocity
        ipos@yarp.IPositionControl;    % IPositionControl interface for joint position control settings
        ivel@yarp.IVelocityControl;    % IVelocityControl interface for joint velocity control settings
        ipwm@yarp.IPWMControl;         % IPWMControl interface for motor PWM control settings
        yarpVector@yarp.Vector;   % Temp buffer vector yarp.Vector of same size as 'jointsList' for read/write purposes
    end
    
    methods
        % Constructor
        function obj = RemoteControlBoardRemapper(robotModel,portsPrefix)
            % Create YARP Network device, to initialize YARP classes for communication
            if ~yarp.Network.initialized
                yarp.Network.init();
            end
            
            % Save robot model
            obj.robotModel = robotModel;
            
            % Create a RemoteControlBoardRemapper device
            % for controlling just the torso+head chain
            % (see http://www.yarp.it/classyarp_1_1dev_1_1remoteControlBoardRemapper.html)
            obj.options = yarp.Property('(device remotecontrolboardremapper)');
            
            % Add port prefix
            obj.options.put('localPortPrefix',['/' portsPrefix]);
            
            % Create a bottle with a list of the axis names
            obj.axesNames = yarp.Bottle();
            obj.axesList = obj.axesNames.addList();
            
            % Create a bottle with a list of the axis control boards
            obj.remoteControlBoards = yarp.Bottle();
            obj.remoteControlBoardsList = obj.remoteControlBoards.addList();
            
            % Initialize other values
            obj.yarpVector = yarp.Vector();
        end
        
        % Destructor
        function delete(obj)
            obj.close();
        end
        
        % Open ports
        open(obj,partList,jointsList)
        
        % Close ports
        close(obj);
        
        % Read/write joint encoders
        [readEncs] = getEncoders(obj);
        ok         = setEncoders(obj,desiredPosMat,refType,refParamsMat,wait,varargin);
        
        % Wait for motion to be completed
        ok = waitMotionDone(obj,timeout);
        
        % Read/write joint velocities
        [readEncSpeeds] = getEncoderSpeeds(obj);
        ok              = setJointRefAccelerations(obj,refAccelerations);
        ok              = velocityMove(obj,desiredVelocities);
        
        % Read/write joint accelerations
        [readEncAccs] = getEncoderAccelerations(obj);
        
        % Read motor encoders
        [readEncs,timeEncs] = getMotorEncoders(obj,motorsIdxList);
        
        % Read motor encoder speeds
        [readEncSpeeds] = getMotorEncoderSpeeds(obj,motorsIdxList);
        
        % Get joints indexes as per the control board remapper mapping
        [jointsIdxList,matchingBitmap] = getJointsMappedIdxes(obj,jointNameList);
        
        % Get number of axes (joints or motors)
        [nbAxes] = getAxes(obj);
        
        % Get joint names from mapped indexes
        [jointsNames] = getJointsNames(obj,jointIdxList);
        
        % Get motor names from mapped indexes
        [motorsNames] = getMotorsNames(obj,motorIdxList);
        
        % Get motors indexes as per the control board remapper mapping
        [motorsIdxList,matchingBitmap] = getMotorsMappedIdxes(obj,motorNameList);
        
        % Get/set control mode for a set of joint indexes. Supported modes are:
        % Position, Open loop (applicable for PWM, torque, current).
        [ok, modes] = getJointsControlMode(obj,jointsIdxList);
        ok          = setJointsControlMode(obj,jointsIdxList,mode);
        
        % Get motor PIDs
        [readPids,readPidsMat] = getMotorsPids(obj,pidCtrlMode,motorsIdxList);
        
        % Get/set the desired PWM values (0-100%) for a set of motor indexes 
        % (for calibration purpose). The motor indexes are the same as for the
        % joints. There is no concept of coupled motors in the control board
        % remapper. But if a coupled motor is set to a given control mode,
        % then all the motors in the coupling are set to the same mode.
        [pwmVec] = getMotorsPWM(obj,motorsIdxList);
        ok       = setMotorsPWM(obj,motorsIdxList,pwmVec);
        
        % Set the desired PWM value (0-100%) for the named motor
        ok = setMotorPWM(obj,motorName,pwm);
        
        % Get the torque values for a set of joint indexes
        [torqVecMat] = getJointTorques(obj,jointsIdxList);
    end
    
    methods(Static = true)
        % matlab <-> yarp c++ map functions
        function matArray = toMatlab(self)
            matArray = str2num(self.toString_c());
        end
        
        function fromMatlab(self,matArray)
            for iter = 1:length(matArray)
                self.set(iter-1,matArray(iter));
            end
        end
    end
    
end

