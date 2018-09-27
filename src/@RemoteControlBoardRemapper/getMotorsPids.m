function [readPids,readPidsMatArray] = getMotorsPids(obj,pidCtrlType,motorsIdxList)

% Get the encoders values
ipids = obj.driver.viewIPidControl();
readPids = yarp.PidVector();
readPids.resize(length(motorsIdxList));
readPid = yarp.Pid();
% readPidsMatArray(1:length(motorsIdxList)) = ...
%     struct('kp',0,'kd',0,'ki',0,'max_int',0,'max_output',0,'scale',1);

% Translate requested PID type in vocab
pidTypeVocab = obj.vocab2pidType(pidCtrlType);

% get fullscale list from the joints database
motorsNameList = obj.getMotorsNames(motorsIdxList);
[~,fullscaleVec] = obj.robotModel.jointsDbase.getMotorGearboxRatioNfullscale(motorsNameList);
fullscaleVec = cell2mat([fullscaleVec{:}]);

% Read PID values for each joint/motor and format the data as a structure
% array
% readPidsMatArray = repmat(struct(...
%     'Kp',double(0),'Kd',double(0),'Ki',double(0),...
%     'max_int',double(0),'max_output',double(0),'scale',double(0)),[1,length(motorsIdxList)]);

% TO BE FIXED
for idx = 1:length(motorsIdxList)
    cLikeIdx = idx-1; % C like index
%     ipids.getPid(pidTypeVocab,cLikeIdx,readPid); % (*)
%     readPids.setbrace(readPid,cLikeIdx);
%     readPidsMatArray(idx) = struct(...
%         'Kp',readPid.kp,'Kd',readPid.kd,'Ki',readPid.ki,...
%         'max_int',readPid.max_int,'max_output',readPid.max_output,...
%         'scale',readPid.scale);
    readPidsMatArray(idx) = struct(...
        'Kp',-4,'Kd',0,'Ki',-5,...
        'max_int',10,'max_output',10,...
        'scale',1);
    
%     % these PID constants were defined for PWM control in dutycycle units, not
%     % fullscale percentage units. So we need to convert these coefficients
%     readPidsMatArray(idx) = structfun(...
%         @(field) field*100/fullscaleVec(idx),readPidsMatArray(idx),'Uniformoutput',false);
    readPidsMatArray(idx).scale = 1;
end

end

% (*) 'readPid' should have been a pointer to a yarp.Pid instead a of a
% yarp.Pid. Probably Swig replaces it with swig_this(readPid) which returns
% the pointer on the object 'readPid'.
