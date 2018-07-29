classdef SensorDataAcquisition
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(Constant=true, Access=public)
        task@char = 'sensorDataAcquisition';
        
        initSection@char = 'sensorsTestDataAcq';
    end
    
    methods(Static = true, Access = public)
        acqSensorDataAccessor = ...
            acquireSensorData(task,taskSpecificParams,robotModel,dataPath,calibedParts);
        
        seqParams = setValFromGrid(gridBuilder,gridParams,acqVel,transVel,labels,robotModel);
    end
    
    methods(Static = true, Access = protected)
        [seqHomeParams,seqEndParams,selector] = getSeqProfile(task,taskSpecificParams,robotModel);
    end
end

