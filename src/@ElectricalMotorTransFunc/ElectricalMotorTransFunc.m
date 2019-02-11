classdef ElectricalMotorTransFunc < handle
    %This class groups all the electrical motor transfer function parameters
    %   The motor current equation is:
    %   i_m = k_pwm2i * PWM + k_bemf * dq_m + i_offset
    
    properties(Constant,Access=public)
        motorName2motorID = containers.Map(...
            {'l_hip_pitch_m','l_hip_roll_m','l_hip_yaw_m','l_knee_m','l_ankle_pitch_m','l_ankle_roll_m',...
            'r_hip_pitch_m','r_hip_roll_m','r_hip_yaw_m','r_knee_m','r_ankle_pitch_m','r_ankle_roll_m',...
            'torso_m1','torso_m2','torso_m3',...
            'l_shoulder_m1','l_shoulder_m2','l_shoulder_m3','l_elbow_m',...
            'r_shoulder_m1','r_shoulder_m2','r_shoulder_m3','r_elbow_m'},...
            {'3B6M0','3B6M1','3B5M0','3B5M1','3B7M0','3B7M1',...
            '3B9M0','3B9M1','3B8M0','3B8M1','3B10M0','3B10M1',...
            '0B3M0','0B3M1','0B4M0',...
            '1B0M0','1B0M1','1B1M0','1B1M1',...
            '2B0M0','2B0M1','2B1M0','2B1M1'});
    end
    
    properties(GetAccess=public, SetAccess=protected)
        name@char;         % motor name
        k_pwm2i = 0;       % k_{pwm,i}
        k_bemf = 0;        % k_{bemf}
        i_offset = 0;      % i_{offset}
        kpwm2iUnits@char;  % k_{pwm,i} units
        kbemfUnits@char;   % k_{bemf} units
        ioffsetUnits@char; % i_{offset} units
    end
    
    methods(Access=protected)
        % Constructor
        function obj = ElectricalMotorTransFunc(motorName,kpwm2iUnits,kbemfUnits,ioffsetUnits)
            obj.name = motorName;
            if nargin == 4
                obj.kpwm2iUnits = kpwm2iUnits;
                obj.kbemfUnits = kbemfUnits;
                obj.ioffsetUnits = ioffsetUnits;
            else
                obj.kpwm2iUnits = '\frac{A}{dutycycle}';
                obj.kbemfUnits = '\frac{A}{rad \cdot s^{-1}}';
                obj.ioffsetUnits = 'A';
            end
        end
    end
    
    methods(Static=true, Access=public)
        function transFunc = GetMotorTransFunc(motorName,calibrationMap)
            if ~isKey(calibrationMap,motorName)
                calibrationMap(motorName) = ElectricalMotorTransFunc(motorName);
            end
            % Return the handle on the newly created or already existing
            % ElectricalMotorTransFunc object.
            transFunc = calibrationMap(motorName);
            % Check if the type is valid
            if (~isa(transFunc,'ElectricalMotorTransFunc') ...
                    && ~isa(transFunc,'FullMotorTransFunc'))
                error('Wrong calibrationMap element class!!');
            end
        end
        
        function isEq = eq(aTransFunc,anotherTransFunc)
            isEq = true;
            for field = fieldnames(aTransFunc)'
                if ischar(aTransFunc.(field{1}))
                    isEq = isEq && strcmp(aTransFunc.(field{1}),anotherTransFunc.(field{1}));
                elseif isnumeric(aTransFunc.(field{1}))
                    isEq = isEq && aTransFunc.(field{1}) == anotherTransFunc.(field{1});
                end
            end
        end
        
        function printFullCalibMap(calibrationMap,part)
            switch part
                case 'left_leg'
                    keyList = {'l_hip_pitch_m','l_hip_roll_m','l_hip_yaw_m','l_knee_m','l_ankle_pitch_m','l_ankle_roll_m'};
                case 'right_leg'
                    keyList = {'r_hip_pitch_m','r_hip_roll_m','r_hip_yaw_m','r_knee_m','r_ankle_pitch_m','r_ankle_roll_m'};
                case 'torso'
                    keyList = {'torso_m1','torso_m2','torso_m3'};
                case 'left_arm'
                    keyList = {'l_shoulder_m1','l_shoulder_m2','l_shoulder_m3','l_elbow_m'};
                case 'right_arm'
                    keyList = {'r_shoulder_m1','r_shoulder_m2','r_shoulder_m3','r_elbow_m'};
                otherwise
            end
            
            includeHeader = true;
            composedString = ''; composedValues = '';
            
            for calibElem = calibrationMap.values(keyList)
                [elemString,values] = calibElem{:}.print1(includeHeader);
                composedString = [composedString elemString];
                composedValues = [composedValues values];
                if includeHeader, includeHeader=false; end
            end
            
            sprintf(composedString,composedValues{:})
        end
        
        function  setUnits(calibrationMap)
            for elem = calibrationMap.values
                elem{:}.kpwm2iUnits = '\frac{A}{dutycycle}';
                elem{:}.kbemfUnits = '\frac{A}{rad \cdot s^{-1}}';
                elem{:}.ioffsetUnits = 'A';
            end
        end
    end
    
    methods(Access=public)
        function setKpwm2i(obj,k_pwm2i), obj.k_pwm2i = k_pwm2i; end
        
        function setKbemf(obj, k_bemf), obj.k_bemf = k_bemf; end
        
        function setIoffset(obj,i_offset), obj.i_offset = i_offset; end
        
        function convertFromOldFormat(obj)
            obj.k_pwm2i = obj.Kpwm;
            obj.k_bemf = obj.Kbemf;
            obj.i_offset = obj.offset;
        end
        function copyObj = copy(obj)
            copyObj = ElectricalMotorTransFunc(obj.name);
            copyObj.k_pwm2i = obj.k_pwm2i;
            copyObj.k_bemf = obj.k_bemf;
            copyObj.i_offset = obj.i_offset;
        end
        
        % Print the parameters of a single motor following the format 1:
        % | motor | $k_{PWM}\ [\frac{A}{dutycycle}] $ | $k_{bemf}\ [\frac{A}{rad \cdot s^{-1}}]$ |  $ i_{offset}\ [A] $ |
        % | <motor-name> | <x.xxxx> |  <x.xxxx> | <x.xxxx> |
        function [aString,values] = print1(obj,includeHeader)
            if includeHeader
                aString = [...
                    '| joint | motor | $k_{pwm,i}\\ [', strrep(obj.kpwm2iUnits,'\','\\'), ...
                    ']$ | $k_{bemf}\\ [', strrep(obj.kbemfUnits,'\','\\'), ...
                    ']$ |  $i_{offset}\\ [', strrep(obj.ioffsetUnits,'\','\\'), ']$ |\n',...
                    '| :-------------: | :-------------: | :-------------: | :-------------: | :-------------: |\n'];
            else
                aString = '';
            end
            aString = [aString ' | `' strrep(obj.name,'_m','') '` | `' ElectricalMotorTransFunc.motorName2motorID(obj.name) '` | %0.4f | %0.4f | %0.4f |\n'];
            values = {obj.k_pwm2i,obj.k_bemf,obj.i_offset};
        end
    end
    
end

