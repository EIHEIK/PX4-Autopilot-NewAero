%% 6. 输出翔仪与PX4典型飞行参数对比表
% 离地统一定义为AGL首次达到1 m且TAS不低于20 m/s。

typicalParameterMetrics = table;

for caseIndex = 1:numel(flightCases)
    one = flightCases(caseIndex);
    track = trackResults(caseIndex);
    groundSpeedMps = hypot(one.VelocityNEDMps(:, 1), ...
        one.VelocityNEDMps(:, 2));
    groundRollStartIndex = find(one.TASMps >= 5 & ...
        one.HeightAGLM <= 1.5, 1, "first");
    liftoffIndex = find(one.HeightAGLM >= 1.0 & ...
        one.TASMps >= 20, 1, "first");

    if isempty(groundRollStartIndex) || isempty(liftoffIndex)
        liftoffTAS = NaN; liftoffGS = NaN; liftoffPitch = NaN;
        takeoffRunTime = NaN; takeoffRunDistance = NaN;
    else
        liftoffTAS = one.TASMps(liftoffIndex);
        liftoffGS = groundSpeedMps(liftoffIndex);
        liftoffPitch = one.PitchDeg(liftoffIndex);
        takeoffRunTime = one.TimeS(liftoffIndex) - ...
            one.TimeS(groundRollStartIndex);
        takeoffRunDistance = hypot( ...
            track.EastM(liftoffIndex) - track.EastM(groundRollStartIndex), ...
            track.NorthM(liftoffIndex) - track.NorthM(groundRollStartIndex));
    end

    [~, peakHeightIndex] = max(one.HeightAGLM);
    touchdownRelative = find(one.HeightAGLM(peakHeightIndex:end) <= 1.0 & ...
        one.TASMps(peakHeightIndex:end) >= 10, 1, "first");
    if isempty(touchdownRelative)
        touchdownIndex = []; touchdownTAS = NaN; touchdownPitch = NaN;
    else
        touchdownIndex = peakHeightIndex + touchdownRelative - 1;
        touchdownTAS = one.TASMps(touchdownIndex);
        touchdownPitch = one.PitchDeg(touchdownIndex);
    end

    airborne = track.Airborne;
    cruiseMask = airborne & one.HeightAGLM >= 50 & ...
        abs(one.RollDeg) <= 5 & abs(one.VerticalSpeedMps) <= 0.5;
    if nnz(cruiseMask) < 10
        cruiseTAS = NaN; cruiseHeight = NaN;
        cruiseHeightStd = NaN; cruisePitchStd = NaN;
    else
        cruiseTAS = mean(one.TASMps(cruiseMask), "omitnan");
        cruiseHeight = mean(one.HeightAGLM(cruiseMask), "omitnan");
        cruiseHeightStd = std(detrend(one.HeightAGLM(cruiseMask)), ...
            "omitnan");
        cruisePitchStd = std(detrend(one.PitchDeg(cruiseMask)), "omitnan");
    end

    routeReferenceHeight = interp1(routeCumulative, routeAltitude, ...
        track.ProgressM, "linear", NaN);
    turnMask = airborne & abs(one.RollDeg) >= 10 & ...
        isfinite(routeReferenceHeight);
    if nnz(turnMask) < 10
        turnHeightErrorMean = NaN; turnMaxHeightLoss = NaN;
    else
        turnHeightError = one.HeightAGLM(turnMask) - ...
            routeReferenceHeight(turnMask);
        turnHeightErrorMean = mean(turnHeightError, "omitnan");
        turnMaxHeightLoss = max(-turnHeightError, [], "omitnan");
    end

    if isempty(liftoffIndex) || isempty(touchdownIndex)
        airborneDuration = NaN;
    else
        airborneDuration = one.TimeS(touchdownIndex) - one.TimeS(liftoffIndex);
    end
    absRollMax = max(abs(one.RollDeg(airborne)), [], "omitnan");
    crossTrackRMS = sqrt(mean(track.CrossTrackM(airborne).^2, "omitnan"));

    typicalParameterMetrics = [typicalParameterMetrics; table( ...
        one.Source, one.Name, liftoffTAS, liftoffGS, liftoffPitch, ...
        takeoffRunTime, takeoffRunDistance, airborneDuration, ...
        nnz(cruiseMask), cruiseTAS, cruiseHeight, cruiseHeightStd, ...
        cruisePitchStd, absRollMax, nnz(turnMask), ...
        turnHeightErrorMean, turnMaxHeightLoss, crossTrackRMS, ...
        touchdownTAS, touchdownPitch, ...
        VariableNames=["Source", "Flight", "LiftoffTASMps", ...
        "LiftoffGroundSpeedMps", "LiftoffPitchDeg", ...
        "TakeoffRunTimeS", "TakeoffRunDistanceM", "AirborneDurationS", ...
        "CruiseSamples", "CruiseTASMeanMps", "CruiseHeightMeanM", ...
        "CruiseHeightDetrendStdM", "CruisePitchDetrendStdDeg", ...
        "AbsRollMaxDeg", "TurnSamples", "TurnHeightErrorMeanM", ...
        "TurnMaxHeightLossM", "CrossTrackRMSM", ...
        "TouchdownTASMps", "TouchdownPitchDeg"])]; %#ok<AGROW>
end

typicalParameterBySource = groupsummary(typicalParameterMetrics, ...
    "Source", ["mean", "std"], ...
    typicalParameterMetrics.Properties.VariableNames(3:end));
writetable(typicalParameterMetrics, fullfile(closedLoopOutputDir, ...
    "typical_flight_parameters.csv"));
writetable(typicalParameterBySource, fullfile(closedLoopOutputDir, ...
    "typical_flight_parameters_by_source.csv"));

disp("典型飞行参数逐架次对比：")
disp(typicalParameterMetrics)
disp("典型飞行参数按来源汇总：")
disp(typicalParameterBySource)
