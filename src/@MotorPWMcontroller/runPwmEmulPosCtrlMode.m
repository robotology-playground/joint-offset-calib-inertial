function [ ok ] = runPwmEmulPosCtrlMode( obj,samplingPeriod,timeout )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% Get the PID parameters (as a structure array)
[~,obj.pidGains] = obj.remCtrlBoardRemap.getMotorsPids('posPID',obj.posCtrledMotors.idx);

% Define and run the position control emulator thread with a discrete PID
% controller.
obj.ctrllerThreadPeriod = samplingPeriod; % thread period
aFilter = DSP.IdentityFilter([]);         % define the filter
% DEBUG
[obj.pidGains.Ki] = deal(-6);
[obj.pidGains.Kp] = deal(-4);
% DEBUG
PIDCtrller = DSP.PIDcontroller(...
    [obj.pidGains.Kp],[obj.pidGains.Kd],[obj.pidGains.Ki],...                 % P, I, D gains
    [obj.pidGains.max_int],[obj.pidGains.max_output],[obj.pidGains.scale],... % max integral term and max correction term
    aFilter);                                                                 % discrete PID controller
startFcn  = @(~,~) obj.ctrllerThreadStartFcn(PIDCtrller);
stopFcn   = @(~,~) obj.ctrllerThreadStopFcn();
updateFcn = @(~,~,ctrllerThreadStop) ... % update function
    obj.ctrllerThreadUpdateFcn(ctrllerThreadStop,samplingPeriod,PIDCtrller);

% Use a timer fully synchronised with Yarp as done by the realtime
% synchronizer in Simulink through the WB-toolbox
if obj.testMode 
    obj.ctrllerThread = UT.RateThread_CB(updateFcn,startFcn,stopFcn,'yarp',samplingPeriod,timeout);
else
    obj.ctrllerThread = RateThread(updateFcn,startFcn,stopFcn,'yarp',samplingPeriod,timeout);
end
ok = obj.ctrllerThread.run(false); % run and don't wait for thread termination

end
