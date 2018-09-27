function [readEncAccs] = getMotorEncoderAccelerations(obj,motorsIdxList)

% Get all the encoders values
imotorencs = obj.driver.viewIMotorEncoders();
readAllEncodersAccs = yarp.Vector();
readAllEncodersAccs.resize(length(obj.motorsList));
imotorencs.getMotorEncoderAccelerations(readAllEncodersAccs.data());
readAllEncAccs=RemoteControlBoardRemapper.toMatlab(readAllEncodersAccs);

% select sub vector
readEncAccs=readAllEncAccs(motorsIdxList);

end
