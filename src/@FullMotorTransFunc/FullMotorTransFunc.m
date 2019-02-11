classdef FullMotorTransFunc < handle
    %This class groups all the motor transfer function parameters
    % 
    
    properties(GetAccess=public, SetAccess=protected)
        name@char;
        Kpwm = 0; % PWM to torque coeff. [Nm / dutycycle%]
        Kc = 0; % Coulomb Friction torque [Nm]
        Kv = 0; % Back-emf + mechanical friction coeff. [Nm * (rad/s)^-1]
        stictionP = 0; % Static friction torque (dq > 0)
        stictionN = 0; % Static friction torque (dq < 0)
        i_offset = 0; % Current offset for PWM = 0 [A]
        k_pwm2i = 0; % PWM to current coeff. [A / dutycycle%]
        k_bemf = 0; % Back-emf coeff. [A * (rad/s)^-1]
        k_t = 0; % Current to torque coeff. [Nm / A]
        k_vmech = 0; % Mechanical friction only coeff. [Nm * (rad/s)^-1]
    end
    
    methods(Access=protected)
        % Constructor
        function obj = FullMotorTransFunc(motorName)
            obj.name = motorName;
        end
    end
    
    methods(Static=true, Access=public)
        function transFunc = GetMotorTransFunc(motorName,calibrationMap)
            if ~isKey(calibrationMap,motorName)
                calibrationMap(motorName) = FullMotorTransFunc(motorName);
            end
            % Return the handle on the newly created or already existing
            % FullMotorTransFunc object.
            transFunc = calibrationMap(motorName);
            % Check if the type is valid
            if ~isa(transFunc,'FullMotorTransFunc')
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
    end
    
    methods(Access=public)
        function setKpwm(obj,Kpwm), obj.Kpwm = Kpwm; end
        
        function setFriction(obj, Kc, Kv)
            obj.Kc = Kc;
            obj.Kv = Kv;
        end
        
        function setStiction(obj,stictionP, stictionN)
            obj.stictionP = stictionP;
            obj.stictionN = stictionN;
        end
        
        function setKpwm2i(obj,k_pwm2i), obj.k_pwm2i = k_pwm2i; end
        
        function setKbemf(obj, k_bemf), obj.k_bemf = k_bemf; end
        
        function setIoffset(obj,i_offset), obj.i_offset = i_offset; end
        
        function setKt(obj,k_t), obj.k_t = k_t; end
        
        function setKvmech(obj,k_vmech), obj.k_vmech = k_vmech; end
        
        function computeKt(obj)
            if not(obj.k_pwm2i == 0)
                obj.k_t = obj.Kpwm / obj.k_pwm2i;
            else
                warning('FullMotorTransFunc: Impossible to compute k_t, k_pwm2i=0');
            end
        end
        
        function computeKvmech(obj, GDqM2Jratio)
            obj.k_vmech = obj.Kv * GDqM2Jratio - obj.k_t * obj.k_bemf;
        end
        
        function anotherTransFunc = copy(obj)
            anotherTransFunc = FullMotorTransFunc(obj.motorName);
            % Copy common fields (name, [Kpwm,Kc,Kv,stictionP,stictionN] or [i_offset,k_pwm2i,k_bemf])
            objFields = fieldnames(obj);
            fields2copy = objFields(isfield(obj,objFields));
            for field = fields2copy'
               anotherTransFunc.(cell2mat(field)) = obj.(cell2mat(field));
            end
        end
    end
    
end

