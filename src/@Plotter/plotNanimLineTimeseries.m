function [ figH,animLinesList ] = plotNanimLineTimeseries(...
    figuresHandler,aTitle,aLabel,yLabel,...
    yLegends,lineStyles,lineWidths,colors,...
    markerSymbols,markerSizes,markerEdgeColors,markerFaceColors)
%Plots y1(t), y2(t),...,yn(t) on the same axis.
%   Plots several timeseries functions y(time) with a single y axis,
%   using a default set of colors.
%   figuresHandler: class object handling all the figures (figure list
%   tracking, saving figures,...)
%   aTitle: figure title
%   aLabel: figure label for the figures handler
%   yLabel: y axis units
%   yLegends: cell list 1xJ of legends for all the J animatedLine objects
%   lineStyles: cell list 1xJ of line styles: '-' (default) | '--' | ':' | '-.' | 'none'
%   lineWidths: cell list 1xJ of line widths: 0.5 (default) | positive value
%   colors: cell list 1xJ of line colors: [0 0 0] (default) | RGB triplet | 'r' | 'g' | 'b' | ...
%   markerSymbols:  cell list 1xJ of marker symbols: 'none' (default) | 'o' | '+' | '*' | '.' | ...
%   markerSizes: cell list 1xJ of marker sizes
%   markerEdgeColors: cell list 1xJ of colors: 'auto' (default) | RGB triplet | 'r' | 'g' | 'b' | ...
%   markerFaceColors: cell list 1xJ of colors: 'none' (default) | 'auto' | RGB triplet | 'r' | 'g' | 'b' | ...

% default values
if isempty(colors), colors={}; colors(1:numel(yLegends))={[0 0 0]}; end
if isempty(markerSymbols), markerSymbols={}; markerSymbols(1:numel(yLegends))={'none'}; end
if isempty(markerSizes),   markerSizes={}; markerSizes(1:numel(yLegends))={6}; end
if isempty(markerEdgeColors), markerEdgeColors={}; markerEdgeColors(1:numel(yLegends))={'auto'}; end
if isempty(markerFaceColors), markerFaceColors={}; markerFaceColors(1:numel(yLegends))={'none'}; end

% create figure
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

% Define the animated lines
animLinesList = cellfun(...
    @(lineStyle,lineWidth,color,markerSymbol,markerSize,markerEdgeColor,markerFaceColor,markerCycle) ...
    animatedline(...
    'LineStyle',lineStyle,'LineWidth',lineWidth,'Color',color,'Marker',markerSymbol,...
    'MarkerSize',markerSize,'MarkerEdgeColor',markerEdgeColor,'MarkerFaceColor',markerFaceColor),...
    lineStyles,lineWidths,colors,markerSymbols,markerSizes,markerEdgeColors,markerFaceColors,...
    'UniformOutput',false);

% set the legends
cellfun(@(an,lg) set(an,'DisplayName',lg),animLinesList,yLegends);

end

