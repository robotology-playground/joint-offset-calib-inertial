function figH = plotNFuncTimeseries(...
    figuresHandler,aTitle,aLabel,yLabel,...    % params irrelevant if figH is not empty
    time,Y,yLegends,...                               % params always required
    lineStyles,lineWidth,markerCycle,figH,resetPlot,alphas) % params always required
%Plots y1(t), y2(t),...,yn(t) on the same axis.
%   Plots several timeseries functions y(time) with a single y axis,
%   using a default set of colors.
%   figuresHandler: class object handling all the figures (figure list
%   tracking, saving figures,...)
%   aTitle: figure title
%   aLabel: figure label for the figures handler
%   yLabel: y axis units
%   time: time vector of size N (Nx1 or 1xN)
%   Y: matrix NxJ. Each column j defines all the N samples of a function yj(t)
%   yLegends: legends for all the functions
%   lineStyles: list 1xJ of line styles. for instance {'b.','g.','r.','cx','mx','yx','k-','w-'};
%   lineWidth: plot line thickness
%   markerCycle: distance, in number of samples, between two markers on the plotted data
%   figH: existing figure handler where to plot in
%   resetPlot: erase previous plot (true), or not (false).

% create figure if no valid figure ref is given
if isempty(figH)
    figH = figure('Name',aTitle,'WindowStyle', 'docked');
    
    if ~isempty(figuresHandler)
        figuresHandler.addFigure(figH,aLabel); % Add figure to the figure handler
    end
    
    % If the figure is not docked, use the below command to display it full
    % screen.
    %set(gcf,'PositionMode','manual','Units','normalized','outerposition',[0 0 1 1]);
    title(aTitle,'FontWeight','bold','Interpreter','latex');
    grid on;
    xlabel('Time (sec)');
    ylabel(yLabel);
    set(gca,'FontSize',24);
    lg=legend('Location','BestOutside');
    lg.set('Interpreter','latex');
    
    % Save fugure label
    figH.UserData = aLabel;
else
    figure(figH);
end

% reset existing plot line if required
hold on % keep by default all axes properties
if (isempty(resetPlot) || resetPlot)
    % clear plot line and reset the ColorOrderIndex and LineStyleOrderIndex
    % axes properties to 1.
    set(gca,'Nextplot','replacechildren');
end

if isempty(markerCycle)
    markerCycle = 1;
end

% Define the line colors
timeYnColorsRaw(1,1:size(Y,2)) = {time};
timeYnColorsRaw(2,:) = num2cell(Y,1); % Y is supposed to have the format NsamplesXnFunctions
timeYnColorsRaw(3,:) = lineStyles;
timeYnColors = timeYnColorsRaw(:)';
yLegends = yLegends(1:size(Y,2));

% plot(X1,Y1,S1,X2,Y2,S2,X3,Y3,S3,...) combines the plots defined by
% the (X,Y,S) triples, where the X's and Y's are vectors or matrices 
% and the S's are strings.
p = plot(timeYnColors{:},'MarkerIndices',1:markerCycle:numel(time),'lineWidth',lineWidth);

[p.DisplayName]=deal(yLegends{:});

% Transparency
if exist('alphas','var')
    for idx = 1:numel(alphas)
        p(idx).Color(4)=alphas(idx);
    end
end

hold off

end

