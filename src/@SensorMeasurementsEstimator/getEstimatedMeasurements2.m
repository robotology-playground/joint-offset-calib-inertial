function [ FTsMeas,AccsMeas,GyrosMeas,ThAxAngAccsMeas,ThAxFTsMeas ] = getEstimatedMeasurements2( obj,q,dq,d2q,indexes,gravity )
%Computes the predicted sensor measurements
% q      : subset of joints positions vector
% dq     : subset of joints velocities vector
% d2q    : subset of joints accelerations vector
% indexes: mapping of subvector to the full vector of joints

qAll = zeros(obj.dofs,1);
dqAll = zeros(obj.dofs,1);
d2qAll = zeros(obj.dofs,1);
% Set the target joints
qAll(indexes,1) = q;
dqAll(indexes,1) = dq;
d2qAll(indexes,1) = d2q;

% Run the estimation
[FTsMeas,AccsMeas,GyrosMeas,ThAxAngAccsMeas,ThAxFTsMeas] = obj.getEstimatedMeasurements(qAll,dqAll,d2qAll,gravity);

end

