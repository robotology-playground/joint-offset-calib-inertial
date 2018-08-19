function [readEncSpeeds] = getEncoderSpeeds(obj)

obj.yarpVector.zero();
obj.iencs.getEncoderSpeeds(obj.yarpVector.data());
readEncSpeeds=obj.yarpVector.toMatlab();

end
