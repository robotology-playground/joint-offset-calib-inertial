classdef JointsDbase < DataBase
    %JointsDbase Implements a database with joints parameters
    %   Detailed explanation goes here
    
    properties(Constant = true, Access = protected)
        defaultMaxDq0 = 0;
    end
    
    properties(Access = protected)
        iDynTreeModel;
    end
    
    methods(Access = public)
        % Constructor
        function obj = JointsDbase(iDynTreeModelFromURDF,link2partH,robotYarpPortPrefix)
            % create property names and keys
            propKeyList = {'jointName','cpldMotorSharingIdx'};
            propNameList = {...
                'jointName','iDynObject',...
                'firstAttachedLink','secondAttachedLink','part','DoF','maxDq0',...
                'jmCoupling','idxInCtrlBoardServer','cpldMotorSharingIdx',...
                'gearboxDqM2Jratio','fullscalePWM'};
            propValueList = cell(iDynTreeModelFromURDF.getNrOfJoints(),length(propNameList));
            
            % Set 'propValueList' with the properties from the iDynTree model
            jointIdxList = 0:iDynTreeModelFromURDF.getNrOfJoints()-1;
            joint2coupling = containers.Map('KeyType','char','ValueType','any');
            propValueLineIdx = 1;
            
            for jointIdx = jointIdxList
                % get native parameters
                iDynObject = iDynTreeModelFromURDF.getJoint(jointIdx);
                jointName = iDynTreeModelFromURDF.getJointName(jointIdx);
                firstAttachedLink = iDynTreeModelFromURDF.getLinkName(iDynObject.getFirstAttachedLink());
                secondAttachedLink = iDynTreeModelFromURDF.getLinkName(iDynObject.getSecondAttachedLink());
                DoF = iDynObject.getNrOfDOFs();
                
                % determine to which part the parent link is attached to
                part = link2partH(secondAttachedLink);
                
                % This joint might be coupled with other joints to a set of motors.
                % Retrieve the respective joints/motors coupling information
                isModelOnline = RobotModel.isOnline;
                if (DoF>0 && isModelOnline.value)
                    [parentCoupling,idxInCtrlBoardServer,cpldMotorSharingIdx,gearboxDqM2Jratio,fullscalePWM] = ...
                        JointsDbase.getJMcouplingFromCtrlBoard(joint2coupling,robotYarpPortPrefix,part,jointName);
                else
                    warning('JointsDbase: %s is a fixed joint.',jointName);
                    parentCoupling = [];
                    idxInCtrlBoardServer = [];
                    cpldMotorSharingIdx = '';
                    gearboxDqM2Jratio = [];
                    fullscalePWM = [];
                end
                
                % fill the properties list
                propValueList(propValueLineIdx,:) = {...
                    jointName,iDynObject,...
                    firstAttachedLink,secondAttachedLink,part,DoF,JointsDbase.defaultMaxDq0,...
                    parentCoupling,idxInCtrlBoardServer,cpldMotorSharingIdx,...
                    gearboxDqM2Jratio,fullscalePWM};
                
                % increment pointer
                propValueLineIdx = propValueLineIdx+1;
            end
            
            % create and build database
            obj = obj@DataBase('keys',propKeyList,'names',propNameList,'values',propValueList);
            obj.build();
            
            % save iDynTree sensors object and coupling list
            obj.iDynTreeModel = iDynTreeModelFromURDF;
        end
        
        % Get joints names from a given part
        jointNameList = getJointNames(obj,part);
        
        % Get joints sharing the same indexes as the given motors
        jointNameList = getCpldJointSharingIdx(obj,motorNameList);
        
        % Get motors sharing the same indexes as the given joints
        motorNameList = getCpldMotorSharingIdx(obj,jointNameList);
        
        % Get the gearbox ratios and fullscale values for a given list of motors
        [gearboxDqM2Jratio,fullscalePWM] = getMotorGearboxRatioNfullscale(obj,motorNameList);
        
        % Get the list of joint/motor couplings. Input type is 'joints'
        % or 'motors'.
        jmCouplings = getJMcouplings(obj,inputType,jointOrMotorNameList);
        
        % Get part names holding the motors
        parts = getPartFromMotors(obj,motorNameList);
        
        % Get the calibration init point Dq0 vector for a given list of joints
        MaxDq0col = getJointsMaxCalibDq0(obj,jointList);
        
        % Get the total DoF from a given list of joints
        DoF = getTotalJointDoF(obj,jointList);
        
        % Get the joint index as mapped in the motors control board server.
        % Axes type is 'joints' or 'motors'.
        [AxesIdxes] = getAxesIdxesFromCtrlBoard(obj,axesType,jointOrMotorNameList);
        
        % Set maximum Dq0 (required by the optimisation solver) for all joints
        success = setAllJointsMaxCalibDq0(obj,maxDq0);
    end
    
    methods(Static=true, Access=protected)
        % Get the joint/motor coupling info from the control board server.
        % The method stores the respective retrieved objects in the running
        % 'joint2coupling', for future queries. It returns the handle of the
        % coupling to which the joint belongs.
        [parentCoupling,idxInCtrlBoardServer,cpldMotorSharingIdx,gearboxDqM2Jratio,fullscalePWM] = ...
            getJMcouplingFromCtrlBoard(joint2coupling,robotYarpPortPrefix,part,jointName);
    end
end

