function [readEncAccs] = getEncoderAccelerations(obj)

obj.yarpVector.zero();
obj.iencs.getEncoderAccelerations(obj.yarpVector.data());
readEncAccs=obj.yarpVector.toMatlab();

end
