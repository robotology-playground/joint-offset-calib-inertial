classdef RemoteControlBoard < handle
    %This class retrieves information from the running control board servers
    %   Detailed explanation goes here
    
    properties(SetAccess = protected, GetAccess = public)
        robotYarpPortPrefix;
        part;
        hardwareMechanicals;
        hwMechanials2robotInterAxesNamesMapping;
        jointsList={};
        robotInterAxesNames;
        options;
        driver;
    end
    
    methods(Access=public)
        % Constructor
        function obj = RemoteControlBoard(robotYarpPortPrefix,part)
            % Create YARP Network device if not yet done, to initialize
            % YARP classes for communication.
            if ~yarp.Network.initialized
                yarp.Network.init();
            end
            % Save parameters
            obj.robotYarpPortPrefix = robotYarpPortPrefix;
            obj.part = part;
            
            % Create the remote control board device associated to the
            % robot part 'part', and open the driver.
            obj.options = yarp.Property('(device remote_controlboard)');
            obj.options.put('remote',['/' robotYarpPortPrefix '/' part]);
            obj.options.put('local',['/AxisInfoCollector/' robotYarpPortPrefix '/' part]);
            % Check that port is registered
            if system(['yarp name query /' robotYarpPortPrefix '/' part '/stateExt:o'])
                warning('Port not registered!! skipping...');
                obj.driver = [];
            else
                obj.driver = yarp.PolyDriver();
                if (~obj.driver.open(obj.options))
                    error(['AxisInfoCollector: Couldn''t open the driver for part ' part]);
                end
            end
            
            % Get config from hardcoded mechanicals config file. This will
            % be compared against the config retrieved from the robot
            % interface.
            hwMechConfigAllParts = Init.load('hardwareMechanicalsConfig');
            obj.hardwareMechanicals = hwMechConfigAllParts.(part);
            
            % Get axes names from the robot interface
            obj.robotInterAxesNames = obj.getAxesNames();
            
            % Get axes (joints) names from config file
            hwMechanialsAxesNames = obj.hardwareMechanicals.jointNames;
            
            % compute bitmap
            [~,obj.hwMechanials2robotInterAxesNamesMapping] = ...
                ismember(hwMechanialsAxesNames,obj.robotInterAxesNames);
            
            % Through warning if order of matching axes differ from the
            % order in the hardwareMechanicalsDevConfig file.
            if ~issorted(obj.hwMechanials2robotInterAxesNamesMapping,'ascend')
                warning(['joint names ordering in the config file differs from ' ...
                'the one given by the robot interface!']);
            end
        end
        
        % Destructor
        function delete(obj)
            if ~isempty(obj.driver)
                obj.driver.close();
            end
        end
        
        % Get number of axes
        function nbAxes = getAxes(obj)
            ipos = obj.driver.viewIPositionControl();
            nbAxes = ipos.getAxes();
        end
        
        % Get Axes list
        function axesNames = getAxesNames(obj)
            % Get joint names from robot interface
            nbAxes = obj.getAxes();
            iaxis = obj.driver.viewIAxisInfo();
            axesNames = cell(1,nbAxes);
            for axisIdx = 1:nbAxes
                axesNames{1,axisIdx} = iaxis.getAxisName(axisIdx-1);
            end
        end
        
        % Get Motors list from 'mechanicals' config file
        function motorNames = getMotorNames(obj)
            % Motor names from axes not listed in the config file will
            % be set from 'robotInterAxesNames'
            dfltMotorNamesVec = cellfun(@(elem) [elem '_m'],obj.robotInterAxesNames,'UniformOutput',false);
            motorNames = obj.setFullFromSubvector(obj.hardwareMechanicals.motorNames,dfltMotorNamesVec);
        end
        
        % Get the digested coupling information
        couplingList = getCouplings(obj);
        
        % Get joints indexes as per the control board server mapping
        [jointsIdxList,matchingBitmap] = getJointsMappedIdxes(obj,jointNameList);
    end
    
    methods(Access=protected)
        % Get parameters from 'mechanicals' config file
        rawCouplingInfo = getRawCoupling(obj);
        
        function fullscalePWMs = getFullscalePWMs(obj)
            dfltFullscalePWM = obj.hardwareMechanicals.fullscalePWM{1};
            dfltFullscaleVec = num2cell(ones(size(obj.robotInterAxesNames))*dfltFullscalePWM);
            % Return the mapping from motor names to fullscalePWM values
            fullscalePWMvalues = obj.setFullFromSubvector(obj.hardwareMechanicals.fullscalePWM,dfltFullscaleVec);
            fullscalePWMs = containers.Map(obj.getMotorNames(),fullscalePWMvalues);
        end
        
        function gearboxDqM2Jratios = getGearboxDqM2Jratios(obj)
            dfltGearboxRatio = obj.hardwareMechanicals.Gearbox_M2J{1};
            dfltGearboxRatiosVector = num2cell(ones(size(obj.robotInterAxesNames))*dfltGearboxRatio);
            % Return the mapping from motor names to Gearbox ratios
            gearboxDqM2JratioValues = num2cell(cell2mat(obj.setFullFromSubvector(obj.hardwareMechanicals.Gearbox_M2J,dfltGearboxRatiosVector)).^-1);
            gearboxDqM2Jratios = containers.Map(obj.getMotorNames(),gearboxDqM2JratioValues);
        end
        
        function fullVector = setFullFromSubvector(obj,subVector,defaultFullVector)
            % values not set from subVecor will be set from the default
            % fullVector
            nbAxes = obj.getAxes();
            fullVector(1,1:nbAxes) = defaultFullVector;
            fullVector(1,obj.hwMechanials2robotInterAxesNamesMapping) = subVector;
        end
    end
    
end

