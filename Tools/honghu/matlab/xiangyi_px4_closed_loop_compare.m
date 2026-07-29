%% 翔仪与鸿鹄翼 V8 100 kg 闭环航迹和平飞段对比
% 本文件是顺序执行脚本，不包含自定义函数。
% 数据对齐原则：
%   1. 四架次均投影到同一个QGC任务折线；
%   2. 全航线按累计航线进度对齐，不按绝对时间强行对齐；
%   3. 平飞段必须位于同一航段内部，并排除航点前后300 m；
%   4. 气动模型一致性与飞控/制导闭环差异分别解释。

%% 1. 路径和输出目录
if ispc
    flyHome = "\\wsl.localhost\Ubuntu-22.04\home\fly";
else
    flyHome = "/home/fly";
end

repoRoot = fullfile(flyHome, "PX4-Autopilot-NewAero");
planFile = fullfile(flyHome, "px4_reference_docs", "current", ...
    "模仿XY航线规划.plan");
xiangyiResultFile = fullfile(repoRoot, "analysis_outputs", ...
    "honghu_v8_xiangyi_matlab", "matlab_xiangyi_v8_aero_results.mat");
px4UlogFiles = [ ...
    fullfile(repoRoot, "build", "px4_sitl_default", "rootfs", "log", ...
        "2026-07-28", "16_23_58.ulg"); ...
    fullfile(repoRoot, "build", "px4_sitl_default", "rootfs", "log", ...
        "2026-07-28", "16_35_51.ulg")];
outputDir = fullfile(repoRoot, "analysis_outputs", ...
    "honghu_v8_xiangyi_closed_loop_matlab");

assert(isfile(planFile), "找不到共同任务航线文件。");
assert(isfile(xiangyiResultFile), "找不到翔仪MATLAB气动核对结果。");
assert(all(isfile(px4UlogFiles)), "找不到100 kg PX4重复飞行ULog。");
if ~isfolder(outputDir)
    mkdir(outputDir);
end

trackNames = [ ...
    "xiangyi_flight_2", "xiangyi_flight_3", ...
    "px4_repeat_1", "px4_repeat_2"];
trackColors = [ ...
    0.0000, 0.4470, 0.7410; ...
    0.0000, 0.6200, 0.4500; ...
    0.8500, 0.3250, 0.0980; ...
    0.6350, 0.0780, 0.1840];

disp("闭环对比输出目录：")
disp(outputDir)

%% 2. 读取共同任务航线
planPayload = jsondecode(fileread(planFile));
missionItems = planPayload.mission.items;
routeLatitude = [];
routeLongitude = [];
routeAltitude = [];

for itemNumber = 1:numel(missionItems)
    if iscell(missionItems)
        oneItem = missionItems{itemNumber};
    else
        oneItem = missionItems(itemNumber);
    end
    if ~isfield(oneItem, "params")
        continue
    end
    oneParams = oneItem.params;
    if iscell(oneParams)
        oneParams = cell2mat(oneParams);
    end
    if numel(oneParams) >= 7 && isfinite(oneParams(5)) && ...
            isfinite(oneParams(6)) && abs(oneParams(5)) > 1 && ...
            abs(oneParams(6)) > 1
        routeLatitude(end + 1, 1) = oneParams(5); %#ok<SAGROW>
        routeLongitude(end + 1, 1) = oneParams(6); %#ok<SAGROW>
        routeAltitude(end + 1, 1) = oneParams(7); %#ok<SAGROW>
    end
end

assert(numel(routeLatitude) >= 10, "任务航线有效地理点数量异常。");
referenceLatitude = routeLatitude(1);
referenceLongitude = routeLongitude(1);
metresPerDegreeLatitude = 111111.0;
metresPerDegreeLongitude = 111111.0 * cosd(referenceLatitude);
routeEast = (routeLongitude - referenceLongitude) * ...
    metresPerDegreeLongitude;
routeNorth = (routeLatitude - referenceLatitude) * ...
    metresPerDegreeLatitude;
routePoints = [routeEast, routeNorth];
routeVectors = diff(routePoints, 1, 1);
routeSegmentLengths = sqrt(sum(routeVectors.^2, 2));
routeCumulative = [0; cumsum(routeSegmentLengths)];

disp("共同航线地理点数和总长度 / m：")
disp([numel(routeLatitude), routeCumulative(end)])

%% 3. 读取翔仪两次完整飞行
load(xiangyiResultFile, "allTimeseries");
requiredXiangyiFields = [ ...
    "Flight", "TimeS", "LatitudeDeg", "LongitudeDeg", ...
    "HeightAGLM", "TASMps", "YawDeg", "PitchDeg", "RollDeg", ...
    "VerticalSpeedMps", "AlphaDeg", "BetaDeg", ...
    "PRadS", "QRadS", "RRadS", ...
    "DeltaADeg", "DeltaEDeg", "DeltaRDeg", "DeltaCDeg", ...
    "MappedTableThrottle", "ThrustN"];
assert(all(ismember(requiredXiangyiFields, ...
    string(allTimeseries.Properties.VariableNames))), ...
    "翔仪MATLAB结果缺少闭环对比所需字段。");

tracks = cell(4, 1);
for flightNumber = 1:2
    selectedFlight = "flight_" + string(flightNumber + 1);
    oneFlight = allTimeseries(allTimeseries.Flight == selectedFlight, :);
    oneTrack = table;
    oneTrack.TimeS = oneFlight.TimeS;
    oneTrack.LatitudeDeg = oneFlight.LatitudeDeg;
    oneTrack.LongitudeDeg = oneFlight.LongitudeDeg;
    oneTrack.HeightAGLM = oneFlight.HeightAGLM;
    oneTrack.TASMps = oneFlight.TASMps;
    oneTrack.YawDeg = mod(oneFlight.YawDeg, 360);
    oneTrack.PitchDeg = oneFlight.PitchDeg;
    oneTrack.RollDeg = oneFlight.RollDeg;
    oneTrack.VerticalSpeedMps = oneFlight.VerticalSpeedMps;
    oneTrack.AlphaDeg = oneFlight.AlphaDeg;
    oneTrack.BetaDeg = oneFlight.BetaDeg;
    oneTrack.PDegS = rad2deg(oneFlight.PRadS);
    oneTrack.QDegS = rad2deg(oneFlight.QRadS);
    oneTrack.RDegS = rad2deg(oneFlight.RRadS);
    oneTrack.DeltaADeg = oneFlight.DeltaADeg;
    oneTrack.DeltaEDeg = oneFlight.DeltaEDeg;
    oneTrack.DeltaRDeg = oneFlight.DeltaRDeg;
    oneTrack.DeltaCDeg = oneFlight.DeltaCDeg;
    oneTrack.Throttle = oneFlight.MappedTableThrottle;
    oneTrack.ThrustN = oneFlight.ThrustN;
    tracks{flightNumber} = oneTrack;
end

%% 4. 用MATLAB R2025a直接读取两次PX4 ULog
wantedTopics = [ ...
    "vehicle_global_position", "vehicle_attitude", ...
    "airspeed_validated", "honghu_v8_aero_state", ...
    "honghu_v8_propulsion_state"];

for repeatNumber = 1:2
    disp("读取PX4 ULog：" + px4UlogFiles(repeatNumber))
    ulogObject = ulogreader(px4UlogFiles(repeatNumber));
    topicRows = readTopicMsgs(ulogObject, "TopicNames", wantedTopics);
    topicNameColumn = string(topicRows.TopicNames);

    globalRow = find(topicNameColumn == "vehicle_global_position", 1);
    attitudeRow = find(topicNameColumn == "vehicle_attitude", 1);
    airspeedRow = find(topicNameColumn == "airspeed_validated", 1);
    aeroRow = find(topicNameColumn == "honghu_v8_aero_state", 1);
    propulsionRow = find(topicNameColumn == ...
        "honghu_v8_propulsion_state", 1);
    assert(all(~isempty([globalRow, attitudeRow, airspeedRow, ...
        aeroRow, propulsionRow])), "ULog缺少闭环对比所需主题。");

    globalMessages = topicRows.TopicMessages{globalRow};
    attitudeMessages = topicRows.TopicMessages{attitudeRow};
    airspeedMessages = topicRows.TopicMessages{airspeedRow};
    aeroMessages = topicRows.TopicMessages{aeroRow};
    propulsionMessages = topicRows.TopicMessages{propulsionRow};

    targetTime = seconds(globalMessages.timestamp);
    [targetTime, uniqueTargetIndex] = unique(targetTime, "stable");
    globalMessages = globalMessages(uniqueTargetIndex, :);

    attitudeTime = seconds(attitudeMessages.timestamp);
    [attitudeTime, uniqueAttitudeIndex] = unique(attitudeTime, "stable");
    attitudeQuaternion = attitudeMessages.q(uniqueAttitudeIndex, :);
    for sampleNumber = 2:size(attitudeQuaternion, 1)
        if dot(attitudeQuaternion(sampleNumber - 1, :), ...
                attitudeQuaternion(sampleNumber, :)) < 0
            attitudeQuaternion(sampleNumber, :) = ...
                -attitudeQuaternion(sampleNumber, :);
        end
    end
    interpolatedQuaternion = interp1(attitudeTime, attitudeQuaternion, ...
        targetTime, "linear", "extrap");
    quaternionNorm = sqrt(sum(interpolatedQuaternion.^2, 2));
    interpolatedQuaternion = interpolatedQuaternion ./ ...
        max(quaternionNorm, 1e-12);
    quaternionW = interpolatedQuaternion(:, 1);
    quaternionX = interpolatedQuaternion(:, 2);
    quaternionY = interpolatedQuaternion(:, 3);
    quaternionZ = interpolatedQuaternion(:, 4);
    rollDeg = rad2deg(atan2( ...
        2 * (quaternionW .* quaternionX + quaternionY .* quaternionZ), ...
        1 - 2 * (quaternionX.^2 + quaternionY.^2)));
    pitchDeg = rad2deg(asin(max(-1, min(1, ...
        2 * (quaternionW .* quaternionY - quaternionZ .* quaternionX)))));
    yawDeg = mod(rad2deg(atan2( ...
        2 * (quaternionW .* quaternionZ + quaternionX .* quaternionY), ...
        1 - 2 * (quaternionY.^2 + quaternionZ.^2))), 360);

    airspeedTime = seconds(airspeedMessages.timestamp);
    [airspeedTime, uniqueAirspeedIndex] = unique(airspeedTime, "stable");
    trueAirspeed = interp1(airspeedTime, ...
        airspeedMessages.true_airspeed_m_s(uniqueAirspeedIndex), ...
        targetTime, "linear", "extrap");

    aeroTime = seconds(aeroMessages.timestamp);
    [aeroTime, uniqueAeroIndex] = unique(aeroTime, "stable");
    alphaDeg = interp1(aeroTime, ...
        aeroMessages.alpha_deg(uniqueAeroIndex), ...
        targetTime, "linear", "extrap");
    betaDeg = interp1(aeroTime, ...
        aeroMessages.beta_deg(uniqueAeroIndex), ...
        targetTime, "linear", "extrap");
    bodyRatesRadS = interp1(aeroTime, ...
        aeroMessages.body_rates_frd_rad_s(uniqueAeroIndex, :), ...
        targetTime, "linear", "extrap");
    documentDeflectionsDeg = interp1(aeroTime, ...
        aeroMessages.delta_doc_deg(uniqueAeroIndex, :), ...
        targetTime, "linear", "extrap");

    propulsionTime = seconds(propulsionMessages.timestamp);
    [propulsionTime, uniquePropulsionIndex] = unique( ...
        propulsionTime, "stable");
    filteredThrottle = interp1(propulsionTime, ...
        propulsionMessages.filtered_throttle(uniquePropulsionIndex), ...
        targetTime, "linear", "extrap");
    thrustN = interp1(propulsionTime, ...
        propulsionMessages.thrust_n(uniquePropulsionIndex), ...
        targetTime, "linear", "extrap");

    latitudeDeg = globalMessages.lat;
    longitudeDeg = globalMessages.lon;
    altitudeM = globalMessages.alt;
    validGeographic = isfinite(latitudeDeg) & isfinite(longitudeDeg) & ...
        abs(latitudeDeg) > 1 & abs(longitudeDeg) > 1;
    initialAltitudeSamples = find(validGeographic, ...
        min(500, nnz(validGeographic)), "first");
    groundAltitudeM = median(altitudeM(initialAltitudeSamples));
    heightAGLM = altitudeM - groundAltitudeM;
    smoothedHeight = smoothdata(heightAGLM, "movmean", 21);
    verticalSpeedMps = gradient(smoothedHeight, targetTime);

    oneTrack = table;
    oneTrack.TimeS = targetTime - targetTime(1);
    oneTrack.LatitudeDeg = latitudeDeg;
    oneTrack.LongitudeDeg = longitudeDeg;
    oneTrack.HeightAGLM = heightAGLM;
    oneTrack.TASMps = trueAirspeed;
    oneTrack.YawDeg = yawDeg;
    oneTrack.PitchDeg = pitchDeg;
    oneTrack.RollDeg = rollDeg;
    oneTrack.VerticalSpeedMps = verticalSpeedMps;
    oneTrack.AlphaDeg = alphaDeg;
    oneTrack.BetaDeg = betaDeg;
    oneTrack.PDegS = rad2deg(bodyRatesRadS(:, 1));
    oneTrack.QDegS = rad2deg(bodyRatesRadS(:, 2));
    oneTrack.RDegS = rad2deg(bodyRatesRadS(:, 3));
    oneTrack.DeltaADeg = documentDeflectionsDeg(:, 1);
    oneTrack.DeltaEDeg = documentDeflectionsDeg(:, 2);
    oneTrack.DeltaRDeg = documentDeflectionsDeg(:, 3);
    oneTrack.DeltaCDeg = documentDeflectionsDeg(:, 4);
    oneTrack.Throttle = filteredThrottle;
    oneTrack.ThrustN = thrustN;
    tracks{repeatNumber + 2} = oneTrack;
end

%% 5. 将四架次按任务顺序投影到同一航线
for trackNumber = 1:4
    oneTrack = tracks{trackNumber};
    validTrack = ...
        isfinite(oneTrack.LatitudeDeg) & ...
        isfinite(oneTrack.LongitudeDeg) & ...
        isfinite(oneTrack.HeightAGLM) & ...
        abs(oneTrack.LatitudeDeg) > 1 & ...
        abs(oneTrack.LongitudeDeg) > 1 & ...
        oneTrack.HeightAGLM >= 5;
    oneTrack = oneTrack(validTrack, :);

    eastM = (oneTrack.LongitudeDeg - referenceLongitude) * ...
        metresPerDegreeLongitude;
    northM = (oneTrack.LatitudeDeg - referenceLatitude) * ...
        metresPerDegreeLatitude;
    takeoffDistance = hypot(eastM - routeEast(1), ...
        northM - routeNorth(1));
    takeoffSearchSamples = max(1, floor(numel(takeoffDistance) / 3));
    [~, takeoffIndex] = min(takeoffDistance(1:takeoffSearchSamples));
    oneTrack = oneTrack(takeoffIndex:end, :);
    eastM = eastM(takeoffIndex:end);
    northM = northM(takeoffIndex:end);

    sampleCount = height(oneTrack);
    crossTrackAbsM = nan(sampleCount, 1);
    crossTrackSignedM = nan(sampleCount, 1);
    routeProgressRawM = nan(sampleCount, 1);
    routeSegment = nan(sampleCount, 1);
    currentSegment = 1;

    for sampleNumber = 1:sampleCount
        bestDistance = inf;
        bestSegment = currentSegment;
        bestFraction = 0;
        bestSignedDistance = NaN;
        lastCandidate = min(currentSegment + 2, ...
            numel(routeSegmentLengths));
        for segmentNumber = currentSegment:lastCandidate
            segmentLengthM = routeSegmentLengths(segmentNumber);
            if segmentLengthM <= 0
                continue
            end
            segmentVector = routeVectors(segmentNumber, :);
            relativePoint = [eastM(sampleNumber), northM(sampleNumber)] - ...
                routePoints(segmentNumber, :);
            segmentFraction = dot(relativePoint, segmentVector) / ...
                segmentLengthM^2;
            segmentFraction = max(0, min(1, segmentFraction));
            projectedPoint = routePoints(segmentNumber, :) + ...
                segmentFraction * segmentVector;
            pointDelta = [eastM(sampleNumber), northM(sampleNumber)] - ...
                projectedPoint;
            oneDistance = hypot(pointDelta(1), pointDelta(2));
            oneSignedDistance = (segmentVector(1) * pointDelta(2) - ...
                segmentVector(2) * pointDelta(1)) / segmentLengthM;
            if oneDistance < bestDistance
                bestDistance = oneDistance;
                bestSegment = segmentNumber;
                bestFraction = segmentFraction;
                bestSignedDistance = oneSignedDistance;
            end
        end
        currentSegment = bestSegment;
        crossTrackAbsM(sampleNumber) = bestDistance;
        crossTrackSignedM(sampleNumber) = bestSignedDistance;
        routeProgressRawM(sampleNumber) = ...
            routeCumulative(bestSegment) + ...
            bestFraction * routeSegmentLengths(bestSegment);
        routeSegment(sampleNumber) = bestSegment;
    end

    oneTrack.EastM = eastM;
    oneTrack.NorthM = northM;
    oneTrack.CrossTrackAbsM = crossTrackAbsM;
    oneTrack.CrossTrackSignedM = crossTrackSignedM;
    oneTrack.RouteProgressM = cummax(routeProgressRawM);
    oneTrack.RouteSegment = routeSegment;
    tracks{trackNumber} = oneTrack;
end

%% 6. 单架次航线指标
trackMetricRecords = struct([]);
for trackNumber = 1:4
    oneTrack = tracks{trackNumber};
    oneRecord = struct;
    oneRecord.Track = trackNames(trackNumber);
    oneRecord.Samples = height(oneTrack);
    oneRecord.ElapsedS = oneTrack.TimeS(end) - oneTrack.TimeS(1);
    oneRecord.RouteProgressMaxM = max(oneTrack.RouteProgressM);
    oneRecord.CrossTrackRMSM = sqrt(mean(oneTrack.CrossTrackAbsM.^2));
    oneRecord.CrossTrackP95M = prctile(oneTrack.CrossTrackAbsM, 95);
    oneRecord.CrossTrackMaxM = max(oneTrack.CrossTrackAbsM);
    oneRecord.HeightMinM = min(oneTrack.HeightAGLM);
    oneRecord.HeightMaxM = max(oneTrack.HeightAGLM);
    oneRecord.TASMinMps = min(oneTrack.TASMps);
    oneRecord.TASMaxMps = max(oneTrack.TASMps);
    if isempty(trackMetricRecords)
        trackMetricRecords = oneRecord;
    else
        trackMetricRecords(end + 1) = orderfields( ...
            oneRecord, trackMetricRecords); %#ok<SAGROW>
    end
end
trackMetrics = struct2table(trackMetricRecords);
writetable(trackMetrics, fullfile(outputDir, "track_metrics.csv"));
disp("单架次航线指标：")
disp(trackMetrics)

%% 7. 按50 m航线进度计算两两差异
pairFirst = [1, 3, 1, 1, 2, 2];
pairSecond = [2, 4, 3, 4, 3, 4];
pairLabels = [ ...
    "xiangyi_repeatability", "px4_repeatability", ...
    "xiangyi_2_vs_px4_1", "xiangyi_2_vs_px4_2", ...
    "xiangyi_3_vs_px4_1", "xiangyi_3_vs_px4_2"];
pairMetricRecords = struct([]);

for pairNumber = 1:numel(pairLabels)
    firstTrack = tracks{pairFirst(pairNumber)};
    secondTrack = tracks{pairSecond(pairNumber)};
    commonProgressStart = max(min(firstTrack.RouteProgressM), ...
        min(secondTrack.RouteProgressM));
    commonProgressEnd = min(max(firstTrack.RouteProgressM), ...
        max(secondTrack.RouteProgressM));
    progressGrid = (ceil(commonProgressStart / 50) * 50 : 50 : ...
        floor(commonProgressEnd / 50) * 50)';

    [firstProgress, firstUniqueIndex] = unique( ...
        firstTrack.RouteProgressM, "stable");
    [secondProgress, secondUniqueIndex] = unique( ...
        secondTrack.RouteProgressM, "stable");
    firstEast = interp1(firstProgress, ...
        firstTrack.EastM(firstUniqueIndex), progressGrid);
    secondEast = interp1(secondProgress, ...
        secondTrack.EastM(secondUniqueIndex), progressGrid);
    firstNorth = interp1(firstProgress, ...
        firstTrack.NorthM(firstUniqueIndex), progressGrid);
    secondNorth = interp1(secondProgress, ...
        secondTrack.NorthM(secondUniqueIndex), progressGrid);
    horizontalSeparation = hypot(secondEast - firstEast, ...
        secondNorth - firstNorth);

    comparisonFields = [ ...
        "HeightAGLM", "TASMps", "RollDeg", "PitchDeg", ...
        "AlphaDeg", "BetaDeg"];
    fieldBias = zeros(1, numel(comparisonFields));
    fieldRMSE = zeros(1, numel(comparisonFields));
    for fieldNumber = 1:numel(comparisonFields)
        fieldName = comparisonFields(fieldNumber);
        firstValue = interp1(firstProgress, ...
            firstTrack.(fieldName)(firstUniqueIndex), progressGrid);
        secondValue = interp1(secondProgress, ...
            secondTrack.(fieldName)(secondUniqueIndex), progressGrid);
        oneDifference = secondValue - firstValue;
        fieldBias(fieldNumber) = mean(oneDifference, "omitnan");
        fieldRMSE(fieldNumber) = sqrt(mean(oneDifference.^2, "omitnan"));
    end

    oneRecord = struct;
    oneRecord.Pair = pairLabels(pairNumber);
    oneRecord.ProgressBins = numel(progressGrid);
    oneRecord.ProgressStartM = progressGrid(1);
    oneRecord.ProgressEndM = progressGrid(end);
    oneRecord.HorizontalRMSM = sqrt(mean(horizontalSeparation.^2, ...
        "omitnan"));
    oneRecord.HorizontalP95M = prctile(horizontalSeparation, 95);
    oneRecord.HeightBiasM = fieldBias(1);
    oneRecord.HeightRMSEM = fieldRMSE(1);
    oneRecord.TASBiasMps = fieldBias(2);
    oneRecord.TASRMSEMps = fieldRMSE(2);
    oneRecord.RollBiasDeg = fieldBias(3);
    oneRecord.RollRMSEDeg = fieldRMSE(3);
    oneRecord.PitchBiasDeg = fieldBias(4);
    oneRecord.PitchRMSEDeg = fieldRMSE(4);
    oneRecord.AlphaBiasDeg = fieldBias(5);
    oneRecord.AlphaRMSEDeg = fieldRMSE(5);
    oneRecord.BetaBiasDeg = fieldBias(6);
    oneRecord.BetaRMSEDeg = fieldRMSE(6);
    if isempty(pairMetricRecords)
        pairMetricRecords = oneRecord;
    else
        pairMetricRecords(end + 1) = orderfields( ...
            oneRecord, pairMetricRecords); %#ok<SAGROW>
    end
end

pairMetrics = struct2table(pairMetricRecords);
writetable(pairMetrics, fullfile(outputDir, ...
    "progress_aligned_pair_metrics.csv"));
disp("按航线进度对齐的两两指标：")
disp(pairMetrics)

%% 8. 自动寻找四架次共同的稳定平飞航段
% 平飞筛选阈值刻意比发动机核对的43~46 m/s稍宽，以允许两套TECS控制器
% 在同一航段内存在合理的小幅速度调节。
flatDurationBySegment = nan(numel(routeSegmentLengths), 4);
flatMaskByTrack = cell(4, 1);
for trackNumber = 1:4
    oneTrack = tracks{trackNumber};
    flatMaskByTrack{trackNumber} = ...
        oneTrack.HeightAGLM >= 50 & ...
        oneTrack.TASMps >= 40 & oneTrack.TASMps <= 48 & ...
        abs(oneTrack.RollDeg) <= 5 & ...
        abs(oneTrack.VerticalSpeedMps) <= 0.5 & ...
        abs(oneTrack.QDegS) <= 1.0;
end

for segmentNumber = 1:numel(routeSegmentLengths)
    if routeSegmentLengths(segmentNumber) <= 600
        continue
    end
    innerStart = routeCumulative(segmentNumber) + 300;
    innerEnd = routeCumulative(segmentNumber + 1) - 300;
    for trackNumber = 1:4
        oneTrack = tracks{trackNumber};
        oneMask = flatMaskByTrack{trackNumber} & ...
            oneTrack.RouteSegment == segmentNumber & ...
            oneTrack.RouteProgressM >= innerStart & ...
            oneTrack.RouteProgressM <= innerEnd;
        if nnz(oneMask) >= 2
            oneTime = oneTrack.TimeS(oneMask);
            nominalStep = median(diff(oneTrack.TimeS), "omitnan");
            flatDurationBySegment(segmentNumber, trackNumber) = ...
                nnz(oneMask) * nominalStep;
        end
    end
end

commonFlatDuration = min(flatDurationBySegment, [], 2, "omitnan");
commonFlatDuration(any(isnan(flatDurationBySegment), 2)) = NaN;
commonFlatDuration(commonFlatDuration < 8) = NaN;
[~, sortedFlatSegments] = sort(commonFlatDuration, "descend", ...
    "MissingPlacement", "last");
selectedFlatSegments = sortedFlatSegments( ...
    isfinite(commonFlatDuration(sortedFlatSegments)));
selectedFlatSegments = selectedFlatSegments( ...
    1:min(3, numel(selectedFlatSegments)));
assert(~isempty(selectedFlatSegments), ...
    "四架次没有找到持续8 s以上的共同稳定平飞航段。");

disp("选中的共同平飞航段（MATLAB一基编号）：")
disp(selectedFlatSegments')
disp("各架次有效平飞时长 / s：")
disp(flatDurationBySegment(selectedFlatSegments, :))

%% 9. 平飞段逐架次统计
flatMetricFields = [ ...
    "HeightAGLM", "TASMps", "RollDeg", "PitchDeg", ...
    "AlphaDeg", "BetaDeg", "DeltaADeg", "DeltaEDeg", ...
    "DeltaRDeg", "DeltaCDeg", "Throttle", "ThrustN"];
flatMetricLabels = [ ...
    "HeightM", "TASMps", "RollDeg", "PitchDeg", ...
    "AlphaDeg", "BetaDeg", "AileronDeg", "ElevatorDeg", ...
    "RudderDeg", "CanardDeg", "Throttle", "ThrustN"];
flatRecords = struct([]);

for selectedNumber = 1:numel(selectedFlatSegments)
    segmentNumber = selectedFlatSegments(selectedNumber);
    innerStart = routeCumulative(segmentNumber) + 300;
    innerEnd = routeCumulative(segmentNumber + 1) - 300;
    for trackNumber = 1:4
        oneTrack = tracks{trackNumber};
        oneMask = flatMaskByTrack{trackNumber} & ...
            oneTrack.RouteSegment == segmentNumber & ...
            oneTrack.RouteProgressM >= innerStart & ...
            oneTrack.RouteProgressM <= innerEnd;
        oneRecord = struct;
        oneRecord.Segment = segmentNumber;
        oneRecord.Track = trackNames(trackNumber);
        oneRecord.Samples = nnz(oneMask);
        nominalStep = median(diff(oneTrack.TimeS), "omitnan");
        oneRecord.DurationS = nnz(oneMask) * nominalStep;
        oneRecord.ProgressStartM = min(oneTrack.RouteProgressM(oneMask));
        oneRecord.ProgressEndM = max(oneTrack.RouteProgressM(oneMask));
        for fieldNumber = 1:numel(flatMetricFields)
            fieldName = flatMetricFields(fieldNumber);
            labelName = flatMetricLabels(fieldNumber);
            oneRecord.(labelName + "Mean") = ...
                mean(oneTrack.(fieldName)(oneMask), "omitnan");
            oneRecord.(labelName + "Std") = ...
                std(oneTrack.(fieldName)(oneMask), "omitnan");
        end
        if isempty(flatRecords)
            flatRecords = oneRecord;
        else
            flatRecords(end + 1) = orderfields( ...
                oneRecord, flatRecords); %#ok<SAGROW>
        end
    end
end

flatSegmentMetrics = struct2table(flatRecords);
writetable(flatSegmentMetrics, fullfile(outputDir, ...
    "flat_segment_metrics.csv"));
disp("共同平飞段逐架次统计：")
disp(flatSegmentMetrics)

%% 10. 平飞段翔仪均值与PX4均值差异
flatDifferenceRecords = struct([]);
meanColumns = flatMetricLabels + "Mean";
for selectedNumber = 1:numel(selectedFlatSegments)
    segmentNumber = selectedFlatSegments(selectedNumber);
    segmentRows = flatSegmentMetrics.Segment == segmentNumber;
    xiangyiRows = segmentRows & startsWith( ...
        flatSegmentMetrics.Track, "xiangyi");
    px4Rows = segmentRows & startsWith( ...
        flatSegmentMetrics.Track, "px4");
    oneRecord = struct;
    oneRecord.Segment = segmentNumber;
    oneRecord.RouteLengthM = routeSegmentLengths(segmentNumber);
    oneRecord.CommonFlatDurationS = ...
        commonFlatDuration(segmentNumber);
    for fieldNumber = 1:numel(meanColumns)
        columnName = meanColumns(fieldNumber);
        metricLabel = flatMetricLabels(fieldNumber);
        xiangyiValues = flatSegmentMetrics.(columnName)(xiangyiRows);
        px4Values = flatSegmentMetrics.(columnName)(px4Rows);
        oneRecord.("Xiangyi" + metricLabel) = mean(xiangyiValues);
        oneRecord.("PX4" + metricLabel) = mean(px4Values);
        oneRecord.("PX4MinusXiangyi" + metricLabel) = ...
            mean(px4Values) - mean(xiangyiValues);
        oneRecord.("XiangyiRepeatSpread" + metricLabel) = ...
            abs(diff(xiangyiValues));
        oneRecord.("PX4RepeatSpread" + metricLabel) = ...
            abs(diff(px4Values));
    end
    if isempty(flatDifferenceRecords)
        flatDifferenceRecords = oneRecord;
    else
        flatDifferenceRecords(end + 1) = orderfields( ...
            oneRecord, flatDifferenceRecords); %#ok<SAGROW>
    end
end

flatSourceDifferences = struct2table(flatDifferenceRecords);
writetable(flatSourceDifferences, fullfile(outputDir, ...
    "flat_segment_source_differences.csv"));
disp("共同平飞段：PX4均值减翔仪均值：")
disp(flatSourceDifferences)

%% 11. 绘制ENU航迹图
figTrajectory = figure("Visible", "off", "Color", "white", ...
    "Position", [100, 100, 1100, 850]);
axisTrajectory = axes(figTrajectory);
plot(axisTrajectory, routeEast, routeNorth, "k--", ...
    "LineWidth", 1.2, "DisplayName", "任务折线");
hold(axisTrajectory, "on");
for trackNumber = 1:4
    oneTrack = tracks{trackNumber};
    plotStep = max(1, ceil(height(oneTrack) / 6000));
    plotIndex = 1:plotStep:height(oneTrack);
    plot(axisTrajectory, oneTrack.EastM(plotIndex), ...
        oneTrack.NorthM(plotIndex), ...
        "Color", trackColors(trackNumber, :), "LineWidth", 1.0, ...
        "DisplayName", trackNames(trackNumber));
end
hold(axisTrajectory, "off");
grid(axisTrajectory, "on");
axis(axisTrajectory, "equal");
xlabel(axisTrajectory, "东向 / m");
ylabel(axisTrajectory, "北向 / m");
title(axisTrajectory, "翔仪与V8 100 kg：共同任务ENU航迹");
legend(axisTrajectory, "Location", "best");
exportgraphics(axisTrajectory, fullfile(outputDir, ...
    "trajectory_enu_comparison.png"), "Resolution", 220);
close(figTrajectory);

%% 12. 分别绘制航线进度状态曲线
plotFields = ["HeightAGLM", "TASMps", "PitchDeg", ...
    "DeltaEDeg", "Throttle"];
plotYLabels = ["离地高度 / m", "真空速 / (m/s)", "俯仰角 / deg", ...
    "升降舵PDF角 / deg", "发动机表输入"];
plotFileNames = ["height_vs_progress.png", "tas_vs_progress.png", ...
    "pitch_vs_progress.png", "elevator_vs_progress.png", ...
    "throttle_vs_progress.png"];
plotTitles = ["高度随航线进度", "空速随航线进度", ...
    "俯仰角随航线进度", "升降舵随航线进度", ...
    "发动机输入随航线进度"];

for figureNumber = 1:numel(plotFields)
    figState = figure("Visible", "off", "Color", "white", ...
        "Position", [100, 100, 1200, 650]);
    axisState = axes(figState);
    hold(axisState, "on");
    for trackNumber = 1:4
        oneTrack = tracks{trackNumber};
        plotStep = max(1, ceil(height(oneTrack) / 6000));
        plotIndex = 1:plotStep:height(oneTrack);
        plot(axisState, oneTrack.RouteProgressM(plotIndex), ...
            oneTrack.(plotFields(figureNumber))(plotIndex), ...
            "Color", trackColors(trackNumber, :), ...
            "LineWidth", 0.9, "DisplayName", trackNames(trackNumber));
    end
    hold(axisState, "off");
    grid(axisState, "on");
    xlabel(axisState, "任务累计航线进度 / m");
    ylabel(axisState, plotYLabels(figureNumber));
    title(axisState, plotTitles(figureNumber));
    legend(axisState, "Location", "best", "NumColumns", 2);
    exportgraphics(axisState, fullfile(outputDir, ...
        plotFileNames(figureNumber)), "Resolution", 220);
    close(figState);
end

%% 13. 保存MAT文件和JSON摘要
save(fullfile(outputDir, "closed_loop_comparison_results.mat"), ...
    "tracks", "trackNames", "routePoints", "routeCumulative", ...
    "trackMetrics", "pairMetrics", "flatSegmentMetrics", ...
    "flatSourceDifferences", "selectedFlatSegments");

summary = struct;
summary.generatedBy = "MATLAB R2025a sequential script";
summary.alignment = ...
    "common mission cumulative progress; no absolute-time forcing";
summary.flatSelection = struct( ...
    "minimumHeightM", 50, "tasRangeMps", [40, 48], ...
    "maxAbsRollDeg", 5, "maxAbsVerticalSpeedMps", 0.5, ...
    "maxAbsPitchRateDegS", 1, "waypointMarginM", 300, ...
    "minimumPerTrackDurationS", 8);
summary.trackMetrics = table2struct(trackMetrics);
summary.pairMetrics = table2struct(pairMetrics);
summary.selectedFlatSegments = selectedFlatSegments;
summary.flatSegmentMetrics = table2struct(flatSegmentMetrics);
summary.flatSourceDifferences = table2struct(flatSourceDifferences);
summary.interpretationBoundary = [ ...
    "Trajectory differences include guidance, controller, mass/inertia, " + ...
    "landing-gear and initialization assumptions; they are not an " + ...
    "aerodynamic-table pass/fail test."];

jsonText = jsonencode(summary, "PrettyPrint", true);
writelines(jsonText, fullfile(outputDir, ...
    "closed_loop_comparison_summary.json"));

disp("闭环航迹和平飞段对比完成。")
disp(outputDir)

%% 14. 量化共同直线航段内的纵向周期活动
% 所有信号统一重采样到20 Hz，避免PX4较高日志频率夸大其方差。
activityFields = ["HeightAGLM", "TASMps", "PitchDeg", "DeltaEDeg"];
activityLabels = ["HeightM", "TASMps", "PitchDeg", "ElevatorDeg"];
activityRecords = struct([]);
commonSampleTime = 0.05;

for selectedNumber = 1:numel(selectedFlatSegments)
    segmentNumber = selectedFlatSegments(selectedNumber);
    innerStart = routeCumulative(segmentNumber) + 300;
    innerEnd = routeCumulative(segmentNumber + 1) - 300;
    for trackNumber = 1:4
        oneTrack = tracks{trackNumber};
        activityMask = ...
            oneTrack.RouteSegment == segmentNumber & ...
            oneTrack.RouteProgressM >= innerStart & ...
            oneTrack.RouteProgressM <= innerEnd & ...
            oneTrack.HeightAGLM >= 50 & ...
            oneTrack.TASMps >= 40 & oneTrack.TASMps <= 48 & ...
            abs(oneTrack.RollDeg) <= 5 & ...
            abs(oneTrack.VerticalSpeedMps) <= 0.5 & ...
            abs(oneTrack.QDegS) <= 1;
        activityTime = oneTrack.TimeS(activityMask);
        [activityTime, uniqueActivityIndex] = unique( ...
            activityTime, "stable");
        uniformTime = (activityTime(1):commonSampleTime:activityTime(end))';

        oneRecord = struct;
        oneRecord.Segment = segmentNumber;
        oneRecord.Track = trackNames(trackNumber);
        oneRecord.DurationS = uniformTime(end) - uniformTime(1);
        for fieldNumber = 1:numel(activityFields)
            oneSignal = oneTrack.(activityFields(fieldNumber))(activityMask);
            oneSignal = oneSignal(uniqueActivityIndex);
            uniformSignal = interp1(activityTime, oneSignal, ...
                uniformTime, "linear");
            detrendedSignal = detrend(uniformSignal, 1);
            sampleCountFFT = numel(detrendedSignal);
            spectrum = abs(fft(detrendedSignal)) / sampleCountFFT;
            frequencyHz = (0:sampleCountFFT - 1)' / ...
                (sampleCountFFT * commonSampleTime);
            frequencyMask = frequencyHz >= 0.05 & frequencyHz <= 2.0;
            candidateSpectrum = spectrum;
            candidateSpectrum(~frequencyMask) = -Inf;
            [~, peakIndex] = max(candidateSpectrum);
            metricLabel = activityLabels(fieldNumber);
            oneRecord.(metricLabel + "Std") = ...
                std(detrendedSignal, "omitnan");
            oneRecord.(metricLabel + "P90Range") = ...
                prctile(detrendedSignal, 95) - ...
                prctile(detrendedSignal, 5);
            oneRecord.(metricLabel + "DominantHz") = ...
                frequencyHz(peakIndex);
        end
        if isempty(activityRecords)
            activityRecords = oneRecord;
        else
            activityRecords(end + 1) = orderfields( ...
                oneRecord, activityRecords); %#ok<SAGROW>
        end
    end
end

longitudinalActivity = struct2table(activityRecords);
writetable(longitudinalActivity, fullfile(outputDir, ...
    "longitudinal_activity_metrics.csv"));
disp("统一20 Hz后的纵向周期活动：")
disp(longitudinalActivity)

%% 15. 绘制最长共同平飞航段的时域活动
longestSegment = selectedFlatSegments(1);
innerStart = routeCumulative(longestSegment) + 300;
innerEnd = routeCumulative(longestSegment + 1) - 300;
activityPlotFields = ["HeightAGLM", "TASMps", "PitchDeg", "DeltaEDeg"];
activityPlotLabels = ["去趋势高度 / m", "去趋势空速 / (m/s)", ...
    "去趋势俯仰角 / deg", "去趋势升降舵 / deg"];
activityPlotFiles = ["flat_longest_height_activity.png", ...
    "flat_longest_tas_activity.png", ...
    "flat_longest_pitch_activity.png", ...
    "flat_longest_elevator_activity.png"];
activityPlotTitles = ["共同平飞航段高度活动", "共同平飞航段空速活动", ...
    "共同平飞航段俯仰活动", "共同平飞航段升降舵活动"];

for figureNumber = 1:numel(activityPlotFields)
    figActivity = figure("Visible", "off", "Color", "white", ...
        "Position", [100, 100, 1200, 650]);
    axisActivity = axes(figActivity);
    hold(axisActivity, "on");
    for trackNumber = 1:4
        oneTrack = tracks{trackNumber};
        activityMask = ...
            oneTrack.RouteSegment == longestSegment & ...
            oneTrack.RouteProgressM >= innerStart & ...
            oneTrack.RouteProgressM <= innerEnd & ...
            oneTrack.HeightAGLM >= 50 & ...
            oneTrack.TASMps >= 40 & oneTrack.TASMps <= 48 & ...
            abs(oneTrack.RollDeg) <= 5 & ...
            abs(oneTrack.VerticalSpeedMps) <= 0.5 & ...
            abs(oneTrack.QDegS) <= 1;
        activityTime = oneTrack.TimeS(activityMask);
        oneSignal = oneTrack.(activityPlotFields(figureNumber))( ...
            activityMask);
        [activityTime, uniqueActivityIndex] = unique( ...
            activityTime, "stable");
        oneSignal = oneSignal(uniqueActivityIndex);
        uniformTime = (activityTime(1):commonSampleTime: ...
            activityTime(end))';
        uniformSignal = interp1(activityTime, oneSignal, ...
            uniformTime, "linear");
        plot(axisActivity, uniformTime - uniformTime(1), ...
            detrend(uniformSignal, 1), ...
            "Color", trackColors(trackNumber, :), ...
            "LineWidth", 0.9, "DisplayName", trackNames(trackNumber));
    end
    hold(axisActivity, "off");
    grid(axisActivity, "on");
    xlabel(axisActivity, "航段内时间 / s");
    ylabel(axisActivity, activityPlotLabels(figureNumber));
    title(axisActivity, activityPlotTitles(figureNumber) + ...
        "（统一20 Hz并去线性趋势）");
    legend(axisActivity, "Location", "best", "NumColumns", 2);
    exportgraphics(axisActivity, fullfile(outputDir, ...
        activityPlotFiles(figureNumber)), "Resolution", 220);
    close(figActivity);
end

%% 16. 将纵向活动补入JSON和MAT结果
summary.longitudinalActivity = table2struct(longitudinalActivity);
jsonText = jsonencode(summary, "PrettyPrint", true);
writelines(jsonText, fullfile(outputDir, ...
    "closed_loop_comparison_summary.json"));
save(fullfile(outputDir, "closed_loop_comparison_results.mat"), ...
    "longitudinalActivity", "-append");

disp("纵向周期活动量化完成。")
