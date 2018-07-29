function acqSensorDataAccessor = acquireSensorData(...
    task,taskSpecificParams,robotModel,dataPath,calibedParts)

% task:      this selects the motion sequence parameters
%            (controlled parts, measured sensors)
% robotModel: robot model
% dataPath: main data folder path where the logger will store the sensor data
% calibedParts: parts to be calibrated. This will select which parts to
%               move and which yarp port to collect data from

% Load sequence profile parameters
[seqHomeParams,seqEndParams,selector] = ...
    SensorDataAcquisition.getSeqProfile(task,taskSpecificParams,robotModel);

            
%% Build the Map sequences from input parameters
%
%  - build the homing sequences 'seqHomeParamsMap'
%  - use selector for filtering sequence parameters of requested parts
%  - build the calibrating sequences 'seqParamsMap'
%  - merge the calibrating sequences 'seqParamsMapMerged'

% ==== Build the homing sequences 'seqHomeParamsMap' and the end sequence:

% First, init MotionSequencer static data, clean YARP ports
clear SequenceParams;
SensorDataYarpI.clean();

% Init sequence parameters from data acquisition input configuration
sequenceParams = SequenceParams(calibedParts,selector,seqHomeParams,seqEndParams);

% Build sequences for the motion runner
sequences = sequenceParams.buildMapSequences();


%% Training data acquisition

% Create Yarp data interface. It can create the necessary yarp ports
% for logging the data and holds a method for connecting or disconnecting
% the ports. It also access data previously logged.
logger = SensorDataYarpI(robotModel.robotEnvNames,dataPath);
% Configure callback logger commands
logCmd.sched = @logger.scheduleNewAcquisition;
logCmd.new   = @logger.newLog;
logCmd.close = @logger.closeLog;
logCmd.start = @logger.connect;
logCmd.stop  = @logger.disconnect;

% create motion sequencer with defined sequences
sequencer = MotionSequencer(task,robotModel,sequences,logCmd);

% run sequencer until all data is acquired
acqSensorDataAccessor = sequencer.run();

% The data acquisition is complete. The sequencer has sent a stop request
% to the logger through 'logCmd' callback.
% The control board device is removed and the yarp devices and objects
% are deleted along with the objects in this context.

% print the sensor data log files info
logger.print();

% Acquisition complete!
disp('Sensor data acquisition complete !!!');

end
