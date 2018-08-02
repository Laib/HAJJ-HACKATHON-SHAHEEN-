function helperUpdateFCWDemoDisplay(frame, videoDisplayHandle, bepPlotters, ...
    laneBoundaries, sensor, confirmedTracks, mostImportantObject, positionSelector, velocitySelector, ...
    visObjects, radObjects)
updateFCWDemoVideoDisplay(videoDisplayHandle, frame, laneBoundaries, sensor, confirmedTracks, positionSelector, mostImportantObject);
helperUpdateFCWDemoBirdsEyePlot(bepPlotters, laneBoundaries, visObjects, radObjects, confirmedTracks, positionSelector, velocitySelector, mostImportantObject);

end
function updateFCWDemoVideoDisplay(videoDisplayHandle, frame, laneBoundaries, sensor, confirmedTracks, positionSelector, MIO)
annotatedFrame = helperAnnotateFCWDemoVideoFrame(frame, laneBoundaries, sensor, confirmedTracks, positionSelector, MIO);
if isvalid(videoDisplayHandle)
    set(videoDisplayHandle, 'CData', annotatedFrame);
end
end