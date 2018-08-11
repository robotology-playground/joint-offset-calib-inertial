function [ ok ] = setJointRefAccelerations(obj,refAccelerations)
% refAccelerations: reference accelerations

% Concatenate reference accelerations matrices, as per
% IVelocityControl interface specification.
refAccelerations = [refAccelerations{:}];

if length(refAccelerations) ~= length(obj.jointsList)
    error('wrong input vector size!');
end

% Set reference accelerations
obj.yarpVector.fromMatlab(refAccelerations);
ok = obj.ivel.setRefAccelerations(obj.yarpVector.data());

end
