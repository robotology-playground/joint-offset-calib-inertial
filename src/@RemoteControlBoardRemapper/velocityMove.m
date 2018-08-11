function [ ok ] = velocityMove(obj,desiredVelocities)
% desiredVelocities: desired velocities to set

% Concatenate desired velocities matrices, as per IVelocityControl interface 
% specification. Same for the reference accelerations.
desiredVelocities = [desiredVelocities{:}];

if length(desiredVelocities) ~= length(obj.jointsList)
    error('wrong input vector size!');
end

% Set desired velocities and run the motion
obj.yarpVector.fromMatlab(desiredVelocities);
ok = obj.ivel.velocityMove(obj.yarpVector.data());

end
