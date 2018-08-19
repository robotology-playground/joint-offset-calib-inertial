function close(obj)
%Cleans all interfaces and closes the device driver

% Clean position and velocity read interface
obj.iencs = yarp.IEncoders.empty(); obj.imotorencs = yarp.IMotorEncoders.empty();

% Reset read sinks
obj.yarpVector.resize(0);

% Clean position and velocity control interfaces
obj.ipos = yarp.IPositionControl.empty();
obj.ivel = yarp.IVelocityControl.empty();
obj.ipwm = yarp.IPWMControl.empty();

% Close the device driver
obj.driver.close();

end
