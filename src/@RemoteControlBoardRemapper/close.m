function close(obj)
%Cleans all interfaces and closes the device driver

% Clean position and velocity read interface
obj.iencs = []; obj.imotorencs = [];

% Reset read sinks
obj.yarpVector.resize(0);

% Clean position and velocity control interfaces
obj.ipos = []; obj.ivel = []; obj.ipwm = [];

% Close the device driver
obj.driver.close();

end
