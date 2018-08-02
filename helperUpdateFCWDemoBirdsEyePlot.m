function helperUpdateFCWDemoBirdsEyePlot(bepPlotters, laneBoundaries, visObjects, ...
    radObjects, confirmedTracks, positionSelector, velocitySelector, mostImportantTrack)
visObjPos = getDetectionPositions(visObjects);
radObjPos = getDetectionPositions(radObjects);
trackIDs = {confirmedTracks.TrackID};
trackLabels = cellfun(@num2str, trackIDs, 'UniformOutput', false);
[trackPositions, trackCovariances] = getTrackPositions(confirmedTracks, positionSelector);
trackVelocities = getTrackVelocities(confirmedTracks, velocitySelector);
plotLaneBoundary(bepPlotters.LaneBoundary, laneBoundaries)
plotDetection(bepPlotters.Radar, radObjPos);
plotDetection(bepPlotters.Vision, visObjPos);
plotTrack(bepPlotters.Track, trackPositions, trackVelocities, trackCovariances, trackLabels);
bepPlotters.MIO.MarkerFaceColor = mostImportantTrack.ThreatColor;
plotTrack(bepPlotters.MIO, trackPositions(mostImportantTrack.TrackIndex,:), trackVelocities(mostImportantTrack.TrackIndex,:), trackLabels(mostImportantTrack.TrackIndex));
end
function positions = getDetectionPositions(detections)
if detections.numObjects > 0
    positions = [detections.object(1:detections.numObjects).position];
    positions = positions(1:2,:).';
else
    positions = zeros(0,2);
end
end