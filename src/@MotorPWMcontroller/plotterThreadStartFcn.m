function plotterThreadStartFcn( obj )
%Creates the figure for plotting Tau vs dq
%    Required for friction calibration only.

% Figure parameters
figTitle = 'Motor velocity to torque model';
xLabel = 'Motor velocity (degrees/s)';
yLabel = 'Motor acceleration (degrees/s^2)';
zLabel = 'Motor torque (N.m)';
obj.tempPlot.units = System.Const.Degrees;
xRange = [-60,60]; % degrees/s
yRange = [-100,100]; % degrees/s^2
zRange = [-10,10]; % N.m

% units conversion
obj.tempPlot.convertFromRad = Math.convertFromRadians(obj.tempPlot.units);
obj.tempPlot.convertFromDeg = Math.convertFromDegrees(obj.tempPlot.units);

% create figure
obj.tempPlot.figH = figure('Name',figTitle,'WindowStyle','docked');

% For displaying an undocked figure in full screen, uncomment the line below.
%set(gcf,'PositionMode','manual','Units','normalized','outerposition',[0 0 1 1]);

% title, axes labels, legend
title(figTitle,'Fontsize',16,'FontWeight','bold');
xlabel(xLabel,'Fontsize',12);
ylabel(yLabel,'Fontsize',12);
zlabel(zLabel,'Fontsize',12);
set(gca,'FontSize',12);
% obj.tempPlot.figH.CurrentAxes.XLim = xRange;
% obj.tempPlot.figH.CurrentAxes.YLim = zRange;

view(45,30); % view(az,el)
% axis vis3d;
% axis image;

% grid and hold the figure open for further data input
grid on;

% define the animated line (the actual plot of the data)
an = animatedline;
an.LineStyle = 'none';
an.Marker = 'o';
an.MarkerFaceColor = 'b';
an.Visible = 'on';
an.Selected = 'off';

obj.tempPlot.an = an;

end
