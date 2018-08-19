function sensorList = createSensorList( model,sensorTypes,sensorNames,parentLinkNames,linkSensorHtransforms )

% Load the URDF model from the respective URDF file, create the iDynTree sensors from the input 
% parameters and wrap them in a sensor list (each model attached to a link of the URDF model, ...).
% Supported sensors:
% - ACCELEROMETER,
% - GYROSCOPE,
% - THREE_AXIS_ANGULAR_ACCELEROMETER.
%

sensorList = iDynTree.SensorsList();

for idx = 1:numel(sensorNames)
    switch sensorTypes{idx}
        case iDynTree.ACCELEROMETER
            sensorObj = iDynTree.AccelerometerSensor();
        case iDynTree.THREE_AXIS_ANGULAR_ACCELEROMETER
            sensorObj = iDynTree.ThreeAxisAngularAccelerometerSensor();
        case iDynTree.GYROSCOPE
            sensorObj = iDynTree.GyroscopeSensor();
    end
    
    sensorObj.setName(sensorNames{idx});
    sensorObj.setParentLink(parentLinkNames{idx});
    parentLinkIdx = model.getLinkIndex(parentLinkNames{idx});
    sensorObj.setParentLinkIndex(parentLinkIdx);
    sensorObj.setLinkSensorTransform(linkSensorHtransforms{idx});
    
    sensorList.addSensor(sensorObj);
end

end

