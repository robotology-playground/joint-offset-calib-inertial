% We support two control modes: position and pwm.
% Before this script is run, 'jtmotgrp' is set to the joints/motors group
% being calibrated. Here we are running the Ktau parameter (2nd phase).
% The targeted joint is kept in position control.


%% Home single step sequence

% For limbs and torso calibration
homeCalib.labels = {...
    'ctrl','ctrl','ctrl','ctrl','ctrl','ctrl';...
    'pos','pos','pos','pos','pos','pos';
    'left_arm','right_arm','left_leg','right_leg','torso','head'};
homeCalib.val = {...
    [0 45 0 50 0 0 0],...
    [0 45 0 50 0 0 0],...
    [0 10 0 0 0 0],...
    [0 10 0 0 0 0],...
    [0 0 0],...
    [0 0 0]};

%% Motion sequences
% (a single sequence is intended to move all defined parts synchronously,
% motions from 2 different sequences should be run asynchronously)
% each calibPart should be calibrated within a single sequence.

% define tables for each limb
left_arm_seqParams.labels = {...
    'mode'   ,'ctrl'               ,'ctrl'          ,'meas'     ,'meas'     ,'pwmctrl';...
    'NA'     ,'pos'                ,'vel'           ,'joint'    ,'jtorq'    ,'pwm'    ;...
    'NA'     ,'left_arm'           ,'left_arm'      ,'left_arm' ,'left_arm' ,jtmotgrp };
left_arm_seqParams.val = {...
    'ctrl'   ,homeCalib.val{1}     ,repmat( 4,[1 7]),false      ,false      ,0        ;...
    'ctrl'   ,homeCalib.val{1}     ,repmat( 4,[1 7]),true       ,true       ,0        };

right_arm_seqParams.labels = {...
    'mode'   ,'ctrl'               ,'ctrl'          ,'meas'     ,'meas'     ,'pwmctrl';...
    'NA'     ,'pos'                ,'vel'           ,'joint'    ,'jtorq'    ,'pwm'    ;...
    'NA'     ,'right_arm'          ,'right_arm'     ,'right_arm','right_arm',jtmotgrp };
right_arm_seqParams.val = left_arm_seqParams.val;

left_leg_seqParams.labels = {...
    'mode'   ,'ctrl'               ,'ctrl'          ,'meas'     ,'meas'     ,'pwmctrl';...
    'NA'     ,'pos'                ,'vel'           ,'joint'    ,'jtorq'    ,'pwm'    ;...
    'NA'     ,'left_leg'           ,'left_leg'      ,'left_leg' ,'left_leg' ,jtmotgrp };
left_leg_seqParams.val = {...
    'ctrl'   ,homeCalib.val{3}     ,repmat( 4,[1 6]),false      ,false      ,0        ;...
    'ctrl'   ,homeCalib.val{3}     ,repmat( 4,[1 6]),true       ,true       ,0        };

right_leg_seqParams.labels = {...
    'mode'   ,'ctrl'               ,'ctrl'          ,'meas'     ,'meas'     ,'pwmctrl';...
    'NA'     ,'pos'                ,'vel'           ,'joint'    ,'jtorq'    ,'pwm'    ;...
    'NA'     ,'right_leg'          ,'right_leg'     ,'right_leg','right_leg',jtmotgrp };
right_leg_seqParams.val = left_leg_seqParams.val;

torso_seqParams.labels = {...
    'mode'   ,'ctrl'               ,'ctrl'          ,'meas'     ,'meas'     ,'pwmctrl';...
    'NA'     ,'pos'                ,'vel'           ,'joint'    ,'jtorq'    ,'pwm'    ;...
    'NA'     ,'torso'              ,'torso'         ,'torso'    ,'torso'    ,jtmotgrp };
torso_seqParams.val = {...
    'ctrl'   ,homeCalib.val{5}     ,repmat( 4,[1 3]),false      ,false      ,0        ;...
    'ctrl'   ,homeCalib.val{5}     ,repmat( 4,[1 3]),true       ,true       ,0        };

head_seqParams.labels = {...
    'mode'   ,'ctrl'               ,'ctrl'          ,'meas'     ,'meas'     ,'pwmctrl';...
    'NA'     ,'pos'                ,'vel'           ,'joint'    ,'jtorq'    ,'pwm'    ;...
    'NA'     ,'head'               ,'head'          ,'head'     ,'head'     ,jtmotgrp };
head_seqParams.val = {...
    'ctrl'   ,homeCalib.val{6}     ,repmat( 4,[1 3]),false      ,false      ,0        ;...
    'ctrl'   ,homeCalib.val{6}     ,repmat( 4,[1 3]),true       ,true       ,0        };

% define Home and End sequences for limbs and torso calibration
seqHomeParams{1} = homeCalib;
seqEndParams     = homeCalib;

% Map parts to sequences and params
selector.calibedParts = {...
    'left_arm','right_arm',...
    'left_leg','right_leg',...
    'torso','head'};
selector.calibedSensors = {...
    {'LLTctrl'},{'LLTctrl'},...
    {'LLTctrl'},{'LLTctrl'},...
    {'LLTctrl'},{'LLTctrl'}};
selector.setIdx  = {1,1,1,1,1,1}; % max index must not exceed max index of seqHomePArams
selector.seqParams = {...
    left_arm_seqParams,right_arm_seqParams,...
    left_leg_seqParams,right_leg_seqParams,...
    torso_seqParams,head_seqParams};
