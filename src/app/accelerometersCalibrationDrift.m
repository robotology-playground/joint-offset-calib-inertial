% This script tests the drift of the accelerometers's calibration parameters, namely:
% - the 3x3 calibration matrix,
% - the 3x1 offsets.

% A calibration element for a single sensor is defined as follows:
% 
%   struct with fields:
% 
%     centre: [3×1 double]
%       quat: [4×1 double]
%          R: [3×3 double]
%      radii: [3×1 double]
%          C: [3×3 double]
% 
% We wish, for each sensor, to store a set of all the calibration values in time, as a set of unique timed vectors,
% having 1 vector per unit of time, that unit being 1 hour. Any matrix from the calibration element is converted to a
% vector column wise. We add to each element:
% - the date as 'datetime' class object with format datetime.Format='dd-MMM-uuuu HH:mm:ss' ('datetime' is processed like
% a linear scale in plots),
% - an index, the index 1 being the first calibration done,
% - a duration ('duration' class object)...
% as the range of validity of the measurement or average. For instance if we compute an average of
% elements in a given range of hours, let's say between h1 and h2 the same day, we get an element of same type, with a
% new hour h3 which is the median of h1 and h2, and a range being the difference btw the 'datetime' objects. this field
% default init is the minimal duration of one measurement test, i.e. 10mn. We get the following matrix structure for N
% consecutive calibration vectors (N measurement iterations):
% 
%     centre: [3×N double]
%       quat: [4×N double]
%          R: [9xN double]
%      radii: [3×N double]
%          C: [9xN double]
%   datetime: [1xN object 'datetime']
%   duration: [1xN object 'duration']
% 
% 'duration' objects can be stored in 'duration' arrays and also support sorting, comparison and mathematical
% calculations.
% The structure described above is associated to each sensor name in a Map container `calibrationDatabase` stored in
% a file `calibrationDatabase.mat`.
% 

% Add main folders in Matlab path
run generatePaths.m;

% clear all variables and close all previous figures
clear all
close all
clc
clear classes; %Clear static data
System.clearTimers(); % Clear all timers
import System.Const; % Define constants

%% ======= PARAMETERS ========
% Define folder where to read new calibration elements
% srcFolder = '/Users/nunoguedelha/dev/green-icub-inertial-sensors-calibration-datasets/repeatability-test-10000samples-imposed-offset/06-12-2018_offsetFrom_06_12_2018';
% srcFolder = '/Users/nunoguedelha/dev/green-icub-inertial-sensors-calibration-datasets/repeatability-test-10000samples-imposed-offset/14-12-2018_offsetFrom_06_12_2018';
srcFolder = '/Users/nunoguedelha/dev/green-icub-inertial-sensors-calibration-datasets/repeatability-test-10000samples-imposed-offset/14-12-2018_offsetFrom_14_12_2018';
% srcFolder = '/Users/nunoguedelha/dev/sensors-calib-inertial/data/phd-thesis-data/5-accelerometers-calibration/old-procedure/2018_07_30-iCubGenova04-accCalibOnPole-5timesIn1hour';
% srcFolder = '/Users/nunoguedelha/dev/sensors-calib-inertial/data/phd-thesis-data/5-accelerometers-calibration/old-procedure/2018_11_21-10-iCubGenova04-accCalibOnPole';
% srcFolder = '/Users/nunoguedelha/dev/sensors-calib-inertial/data/phd-thesis-data/5-accelerometers-calibration/new-procedure/offsetsCalibration';
% srcFolder = '/Users/nunoguedelha/dev/sensors-calib-inertial/data/phd-thesis-data/5-accelerometers-calibration/new-procedure/gainsCalibration';
accNames(1,1:8) = {...
    'l_upper_leg_mtb_acc_10b1'
    'l_upper_leg_mtb_acc_10b2'
    'l_upper_leg_mtb_acc_10b3'
    'l_upper_leg_mtb_acc_10b4'
    'l_lower_leg_mtb_acc_10b8'
    'l_lower_leg_mtb_acc_10b9'
    'l_lower_leg_mtb_acc_10b10'
    'l_lower_leg_mtb_acc_10b11'}';

saveDatabase = false;
plotDrift = true;

%% ======= TEST ==============
% Load current database
if exist('calibrationDatabase.mat','file') == 2
    load('calibrationDatabase.mat','calibrationDatabase');
end
if ~exist('calibrationDatabase','var')
    calibrationDatabase = containers.Map('KeyType','char','ValueType','any');
end

% read selected folder for new calibration elements
listOfCalibFiles = dir([srcFolder '/**/calibrationMap*.mat']);

% Process calibrationMap files 1 by 1. Each calibrationMap file provides a calibration column vector, with a date.
for file = {listOfCalibFiles.folder;listOfCalibFiles.name;listOfCalibFiles.date}
    fullpath = [file{1} '/' file{2}];
    load(fullpath,'calibrationMap');
    % Update the database entries one by one
    for accName = accNames
        if (calibrationMap.isKey(accName{1}))
            newAccCalibEntry = calibrationMap(accName{1});
            newAccCalibEntry.R = newAccCalibEntry.R(:);
            newAccCalibEntry.C = newAccCalibEntry.C(:);
            newAccCalibEntry.date = datetime(file{3});
            newAccCalibEntry.duration = duration(0,10,0);
            if (calibrationDatabase.isKey(accName{1}))
                accCalibRecord = calibrationDatabase(accName{1});
            else
                accCalibRecord = [];
            end
            calibrationDatabase(accName{1}) = [accCalibRecord, newAccCalibEntry];
        end
    end
end
clear calibrationMap;

distrCentre = containers.Map();
distrCmatrix = containers.Map();
tablePrint = containers.Map();

% Sort the database
for accName = accNames
    accCalibRecord = calibrationDatabase(accName{1});
    dateVec = [accCalibRecord.date]';
    centreMat = [accCalibRecord.centre]';
    Cmat =[accCalibRecord.C]';
    accShortName = System.toLatexInterpreterCompliant(getShortSensorName(accName{1}));
    
    %% Distributions.
    %  For the matrix C, we only care about the elements xx, yy, zz, yx, zx, zy since it is a symmetric matrix.
    %  We select the elements: [1 5 9 2 3 6].
    centreMat4distr = centreMat';
    Cmat4distr = Cmat(:,[1 5 9 2 3 6])';
    centreMean = mean(centreMat4distr,2);
    centreStd  = std(centreMat4distr,1,2);
    cMatrixMean = mean(Cmat4distr,2);
    cMatrixStd  = std(Cmat4distr,1,2);
    % to be printed on latex document
    meanString = [accShortName,mat2str([centreMean;cMatrixMean],3),' \\'];
    meanString = strrep(meanString,'acc\_','MTB ');
    meanString = strrep(meanString,'[',' & ');
    meanString = strrep(meanString,';',' & ');
    meanString = strrep(meanString,']','');
    stdString = [accShortName,mat2str([centreStd;cMatrixStd],3),' \\'];
    stdString = strrep(stdString,'acc\_','MTB ');
    stdString = strrep(stdString,'[',' & ');
    stdString = strrep(stdString,';',' & ');
    stdString = strrep(stdString,']','');
    tablePrint(accName{1}) = struct('mean',meanString,'std',stdString);
    % for later use
    distrCentre(accName{1}) = struct('mean',centreMean,'std',centreStd);
    distrCmatrix(accName{1}) = struct('mean',cMatrixMean,'std',cMatrixStd);
    
    %% Plots
    if plotDrift
        % Offsets evolution
        figH = Plotter.plotNFuncTimeseries(...
            [],['Drift of ' accShortName ' offsets'],...
            ['drift_' accShortName '_offsets'],'m \cdot s^{-2}',...
            dateVec,centreMat,{[accShortName ' offset$_x$'],[accShortName ' offset $_y$'],[accShortName ' offset $_z$']},...
            {'r-','g-','b-'},4,[],[],[]);
        % Calibration matrix
        figH = Plotter.plotNFuncTimeseries(...
            [],['Drift of ' accShortName ' calibration matrix C'],...
            ['drift_' accShortName '_Cmatrix'],'ratio',...
            dateVec,Cmat(:,[1 5 9 2 3 6]),{...
            [accShortName ' gain $xx$'],[accShortName ' gain $yy$'],[accShortName ' gain $zz$'],...
            [accShortName ' cross gain $yx$'],[accShortName ' cross gain $zx$'],...
            [accShortName ' cross gain $zy$']},...
            {'r-','g-','b-','c-','m-','y-'},4,[],[],[]);
    end
end

if saveDatabase
    save('calibrationDatabase.mat','calibrationDatabase');
end

tablePrintList  = tablePrint.values(accNames);
tablePrintArray = cell2mat(tablePrintList');
% Display
tableMEAN = {tablePrintArray.mean}'
tableSTD = {tablePrintArray.std}'
% save acc 10b4 and 10b11 plots
if plotDrift
%     UI.saveFigures2(srcFolder,[7,8,15,16]);
end


%% ======= Static local functions ============

function shortName = getShortSensorName(fullSensorName)

splitName = textscan(fullSensorName,'%s','delimiter','_');
shortName = [splitName{1}{end-1} '_' splitName{1}{end}];

end
