classdef ElectricalMotorTransFunc < handle
    %This class groups all the electrical motor transfer function parameters
    %   The transfer function parameters are:
    %   - name: motor name
    %   - k_pwm2i: k_{pwm,i}
    %   - k_bemf : k_{bemf}
    %   - i_offset: i_{offset}
    %   The motor current equation is:
    %   i_m = k_pwm2i * PWM + k_bemf * dq_m + i_offset
    
    properties(GetAccess=public, SetAccess=protected)
        name@char;
        k_pwm2i = 0;
        k_bemf = 0;
        i_offset = 0;
    end
    
    methods(Access=protected)
        % Constructor
        function obj = ElectricalMotorTransFunc(motorName)
            obj.name = motorName;
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
    end
    
end

