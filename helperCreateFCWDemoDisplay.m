function [videoReader, videoDisplayHandle, bepPlotters, sensor] = helperCreateFCWDemoDisplay(videoFileName, sensorConfigurationFileName)
videoReader = VideoReader(videoFileName);
height = videoReader.Height;
width  = videoReader.Width;
frame = readFrame(videoReader);
toolBarSize = 60; 
figurePosition = getFigurePosition(height+toolBarSize, width*2); % Twice the size to allow video and bird's-eye plot
f = figure('Position', figurePosition, 'Name', 'Forward Collision Warning With Tracking Example');
hVideoAxes = axes(f, 'Units', 'Normal', 'Position', [0.01 0.01 0.49 0.88]);
videoDisplayHandle = createFCWDemoVideoDisplay(frame, hVideoAxes);
bepAxes = axes(f, 'Units', 'Normal', 'Position', [0.55 0.1 0.44 0.78]);
bepPlotters = helperCreateFCWDemoBirdsEyePlot(sensorConfigurationFileName, bepAxes); 
load('FCWDemoMonoCameraSensor.mat', 'sensor')
videoReader.CurrentTime = 0;
end
function figurePosition = getFigurePosition(height, width)
screenMargin = [0, 100];   % [left, top]
f = figure('Visible', 'off');
defPosition = get(f, 'Position');
figurePosition = [max(defPosition(1) + defPosition(3) - width, screenMargin(1)), max(defPosition(2) + defPosition(4) - height, screenMargin(2)), width, height];
close(f);
end

function videoFrame = createFCWDemoVideoDisplay(frame, hVideoAxes)
videoFrame = imshow(frame, [], 'Parent', hVideoAxes);
h = title('Recorded Video');
set(h, 'Position', [320.5, -10, 0])
end