classdef Buffer < handle
    %Class for holding a buffer to be polled
    %   Detailed explanation goes here
    
    properties(Access=public)
        value; % no type specified as intended
    end
    
    methods
        function obj = Buffer(value)
            obj.value = value;
        end
        
        function theValue = getValue(obj)
            theValue = obj.value;
        end
    end
end

