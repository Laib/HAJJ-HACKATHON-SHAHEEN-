function bepPlotters = helperCreateFCWDemoBirdsEyePlot(sensorConfigurationFile, bepAxes)
if nargin == 1
    bepAxes = [];
end
load(sensorConfigurationFile, 'sensorParams');
BEP = birdsEyePlot('Parent',bepAxes,'XLimits',[0 90],'YLimits',[-35 35]);
cap(1) = coverageAreaPlotter(BEP,'FaceColor','blue','EdgeColor','blue');
cap(2) = coverageAreaPlotter(BEP,'FaceColor','red','EdgeColor','red');
cap(3) = coverageAreaPlotter(BEP,'FaceColor','red','EdgeColor','red');
for i = 1:3
    plotCoverageArea(cap(i), [sensorParams(i).X, sensorParams(i).Y],...
        sensorParams(i).Range, sensorParams(i).YawAngle, sensorParams(i).FoV);
end
bepPlotters.Vision = detectionPlotter(BEP, 'DisplayName','vision detection', ...
    'MarkerEdgeColor','blue', 'Marker','^');
bepPlotters.Radar = detectionPlotter(BEP, 'DisplayName','radar detection', ...
    'MarkerEdgeColor','red');
bepPlotters.Track = trackPlotter(BEP, 'DisplayName','tracked object', ...
    'HistoryDepth',10);
bepPlotters.MIO = trackPlotter(BEP, 'DisplayName','most important object', ...
    'MarkerFaceColor','black');
bepPlotters.LaneBoundary = laneBoundaryPlotter(BEP, ...
    'DisplayName','lane boundaries', 'Color',[.9 .9 0]);
title('Birds-Eye Plot')
set(BEP.Parent.Legend, 'AutoUpdate', 'off'); 
end