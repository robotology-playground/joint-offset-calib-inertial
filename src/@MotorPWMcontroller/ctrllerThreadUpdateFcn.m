function [ ok ] = ctrllerThreadUpdateFcn( obj,ctrllerThreadStop,rateThreadPeriod,PIDCtrller )
%Run the PID controller and set the computed PWM values
%   Detailed explanation goes here

% Get the positions and velocities of the emulated pos controlled motors
[currentMotorsPos,currentMotorsTime] = obj.remCtrlBoardRemap.getMotorEncoders(obj.posCtrledMotors.idx);
currentMotorsVel = obj.remCtrlBoardRemap.getMotorEncoderSpeeds(obj.posCtrledMotors.idx);

% compute ellapsed time
if any(isnan(obj.prevMotorsTime))
    timeStep = ones(size(currentMotorsTime))*rateThreadPeriod;
else
    timeStep = currentMotorsTime - obj.prevMotorsTime;
end
obj.prevMotorsTime = currentMotorsTime;

% Run the PID controller.
% We apply the PID control to the variables measured at the gearbox output,
% i.e. we consider the block [motor+gearbox] as a black box.
gearboxRatios = obj.normOfgearboxDqM2Jratios(~obj.pwmCtrledMotorBitmapInCoupling);
[intSat,outSat,posPwmVec] = PIDCtrller.step(...
    timeStep(:),...
    obj.lastMotorsPosInPrevMode(:).*gearboxRatios(:),...
    currentMotorsPos(:).*gearboxRatios(:),...
    currentMotorsVel(:).*gearboxRatios(:));

if any([intSat,outSat])
    % throw warning
    warning(['PID produced a saturated value (intSat=' num2str(intSat) ',outSat=' num2str(outSat) ') during position control emulation !!']);
end

% Set the computed PWM values
obj.posCtrledMotors.pwm = posPwmVec;
ok = obj.remCtrlBoardRemap.setMotorsPWM(obj.posCtrledMotors.idx,posPwmVec);
ok = ok && obj.remCtrlBoardRemap.setMotorsPWM(obj.pwmCtrledMotor.idx,obj.pwmCtrledMotor.pwm);

if ~ok
    % stop the timer with an error
    ctrllerThreadStop(false);
    % throw error
    warning('PWM setting failed during position control emulation !!');
end

end
