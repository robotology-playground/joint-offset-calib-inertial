function [readEncSpeeds] = getEncoderSpeeds(obj)

obj.yarpVector.zero();
iencs.getEncoderSpeeds(obj.yarpVector.data());
readEncSpeeds=obj.yarpVector.toMatlab();

end
