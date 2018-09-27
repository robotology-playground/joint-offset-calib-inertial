%
% Model parameters for the low level joint torque control calibration
% 
% We can define here information relative to a subset of the axes retrieved
% through the robot interface.
%

% libraries
import containers.Map;

% left arm parameters
axisConfig = {...
    'l_shoulder_pitch' 'l_shoulder_roll' 'l_shoulder_yaw' 'l_elbow'   % AxisName
    'l_shoulder_m1'    'l_shoulder_m2'   'l_shoulder_m3'  'l_elbow_m' % MotorName
    32000               32000            32000            32000       % fullscalePWM
    100.00              100.00           100.00           100.00   }; % Gearbox_M2J

left_arm.jointNames   = axisConfig(1,:);
left_arm.motorNames   = axisConfig(2,:);
left_arm.fullscalePWM = axisConfig(3,:);
left_arm.Gearbox_M2J  = axisConfig(4,:);
left_arm.matrixM2J = {...
   [1.000   0.000   0.000   0.000
    1.000   0.615   0.000   0.000
    0.000  -0.615   0.615   0.000
    0.000   0.000   0.000   1.000]};

% right arm parameters
axisConfig = {...
    'r_shoulder_pitch' 'r_shoulder_roll' 'r_shoulder_yaw' 'r_elbow'   % AxisName
    'r_shoulder_m1'    'r_shoulder_m2'   'r_shoulder_m3'  'r_elbow_m' % MotorName
    32000              32000             32000            32000       % fullscalePWM
    -100.00            -100.00           -100.00          -100.00  }; % Gearbox_M2J

right_arm.jointNames   = axisConfig(1,:);
right_arm.motorNames   = axisConfig(2,:);
right_arm.fullscalePWM = axisConfig(3,:);
right_arm.Gearbox_M2J  = axisConfig(4,:);
right_arm.matrixM2J = {...
   [1.000   0.000   0.000   0.000
    1.000   0.615   0.000   0.000
    0.000  -0.615   0.615   0.000
    0.000   0.000   0.000   1.000]};

% left leg parameters
axisConfig = {...
    'l_hip_pitch'   'l_hip_roll'   'l_hip_yaw'   'l_knee'   'l_ankle_pitch'   'l_ankle_roll'   % AxisName
    'l_hip_pitch_m' 'l_hip_roll_m' 'l_hip_yaw_m' 'l_knee_m' 'l_ankle_pitch_m' 'l_ankle_roll_m' % MotorName
    32000           32000          32000         32000      32000             32000            % fullscalePWM
    -100.0          100.0          -100.0        -100.0     100.0             100.0         }; % Gearbox_M2J

left_leg.jointNames   = axisConfig(1,:);
left_leg.motorNames   = axisConfig(2,:);
left_leg.fullscalePWM = axisConfig(3,:);
left_leg.Gearbox_M2J  = axisConfig(4,:);
left_leg.matrixM2J = {...
   [1.00    0.00    0.00    0.00
    0.00    1.00    0.00    0.00
    0.00    0.00    1.00    0.00
    0.00    0.00    0.00    1.00],...
   [1.00    0.00
    0.00    1.00]};

% right leg parameters
axisConfig = {...
    'r_hip_pitch'   'r_hip_roll'   'r_hip_yaw'   'r_knee'   'r_ankle_pitch'   'r_ankle_roll'   % AxisName
    'r_hip_pitch_m' 'r_hip_roll_m' 'r_hip_yaw_m' 'r_knee_m' 'r_ankle_pitch_m' 'r_ankle_roll_m' % MotorName
    32000           32000          32000         32000      32000             32000            % fullscalePWM
    100.0           -100.0         100.0         100.0      -100.0            -100.0        }; % Gearbox_M2J

right_leg.jointNames   = axisConfig(1,:);
right_leg.motorNames   = axisConfig(2,:);
right_leg.fullscalePWM = axisConfig(3,:);
right_leg.Gearbox_M2J  = axisConfig(4,:);
right_leg.matrixM2J = {...
   [1.00    0.00    0.00    0.00
    0.00    1.00    0.00    0.00
    0.00    0.00    1.00    0.00
    0.00    0.00    0.00    1.00],...
   [1.00    0.00
    0.00    1.00]};

% torso parameters
axisConfig = {...
    'torso_roll' 'torso_pitch' 'torso_yaw'     % AxisName
    'torso_m1'   'torso_m2'    'torso_m3'      % MotorName
    32000        32000         32000           % fullscalePWM
    -100.00      -100.00       -100.00      }; % Gearbox_M2J

torso.jointNames   = axisConfig(1,:);
torso.motorNames   = axisConfig(2,:);
torso.fullscalePWM = axisConfig(3,:);
torso.Gearbox_M2J  = axisConfig(4,:);
torso.matrixM2J = {...
   [0.500    0.500    0.000
   -0.500    0.500    0.000
    0.275    0.275    0.550]};

% head parameters
axisConfig = {...
    'neck_pitch' 'neck_roll' 'neck_yaw'   % AxisName
    'neck_m1'    'neck_m2'   'neck_yaw_m' % MotorName
    3360         3360        3360         % fullscalePWM
    161.68       161.68      100       }; % Gearbox_M2J

head.jointNames   = axisConfig(1,:);
head.motorNames   = axisConfig(2,:);
head.fullscalePWM = axisConfig(3,:);
head.Gearbox_M2J  = axisConfig(4,:);
head.matrixM2J = {...
   [0.500  -0.500
    0.500   0.500],...
   [1.000]};

