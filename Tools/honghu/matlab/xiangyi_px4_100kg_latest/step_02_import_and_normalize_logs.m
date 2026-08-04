%% 2. 读取翔仪与PX4日志，并统一为FRD/NED、20 Hz数据接口

flightCases = struct([]);
caseNumber = 0;

%% 2.1 翔仪CSV：读取577列表头和所需字段
fileID = fopen(xiangyiCsv, "r", "n", "GB18030");
assert(fileID >= 0, "无法打开翔仪CSV。");
headerLine = fgetl(fileID);
fgetl(fileID);
fclose(fileID);
columnNames = string(split(headerLine, ","));
rawXiangyi = readmatrix(xiangyiCsv, "NumHeaderLines", 2, ...
    "Encoding", "GB18030");
if size(rawXiangyi, 2) == 576
    rawXiangyi(:, 577) = NaN;
end
assert(numel(columnNames) == 577 && size(rawXiangyi, 2) == 577, ...
    "翔仪CSV不是预期的577列结构。");

requiredNames = [ ...
    "IndexPro", "Lat", "Lon", "NavHeight", "NavAltitude", ...
    "V_east", "V_north", "Vz", "TAS", ...
    "Yaw", "Pitch", "Roll", "wX", "wY", "wZ", ...
    "Acc_X", "Acc_Y", "Acc_Z", ...
    "FW_Ail", "FW_Ele", "FW_Thr", "FW_Rud", "Canard"];
columnIndex = zeros(size(requiredNames));
for fieldNumber = 1:numel(requiredNames)
    found = find(columnNames == requiredNames(fieldNumber), 1, "first");
    assert(~isempty(found), "翔仪CSV缺少字段：" + requiredNames(fieldNumber));
    columnIndex(fieldNumber) = found;
end
xiangyiData = array2table(rawXiangyi(:, columnIndex), ...
    "VariableNames", cellstr(requiredNames));
xiangyiData.SourceRow = (1:height(xiangyiData))';

% 第一架次存在经纬度跳变；正式统计只使用第二、第三次完整飞行。
xiangyiNames = ["xiangyi_flight_2", "xiangyi_flight_3"];
xiangyiStart = [14636, 31761];
xiangyiEnd = [27805, 46063];

for flightNumber = 1:2
    caseNumber = caseNumber + 1;
    rows = xiangyiStart(flightNumber):xiangyiEnd(flightNumber);
    one = xiangyiData(rows, :);
    sampleCount = height(one);
    timeS = (0:sampleCount-1)' * sampleTimeS;

    rollRad = deg2rad(one.Roll);
    pitchRad = deg2rad(one.Pitch);
    yawRad = unwrap(deg2rad(one.Yaw));
    cRoll = cos(rollRad); sRoll = sin(rollRad);
    cPitch = cos(pitchRad); sPitch = sin(pitchRad);
    cYaw = cos(yawRad); sYaw = sin(yawRad);

    velocityNED = [one.V_north, one.V_east, -one.Vz];
    velocityFRD = zeros(sampleCount, 3);
    velocityFRD(:, 1) = ...
        cPitch .* cYaw .* velocityNED(:, 1) + ...
        cPitch .* sYaw .* velocityNED(:, 2) - ...
        sPitch .* velocityNED(:, 3);
    velocityFRD(:, 2) = ...
        (sRoll .* sPitch .* cYaw - cRoll .* sYaw) .* velocityNED(:, 1) + ...
        (sRoll .* sPitch .* sYaw + cRoll .* cYaw) .* velocityNED(:, 2) + ...
        sRoll .* cPitch .* velocityNED(:, 3);
    velocityFRD(:, 3) = ...
        (cRoll .* sPitch .* cYaw + sRoll .* sYaw) .* velocityNED(:, 1) + ...
        (cRoll .* sPitch .* sYaw - sRoll .* cYaw) .* velocityNED(:, 2) + ...
        cRoll .* cPitch .* velocityNED(:, 3);

    alphaRad = atan2(velocityFRD(:, 3), velocityFRD(:, 1));
    betaRad = atan2(velocityFRD(:, 2), ...
        hypot(velocityFRD(:, 1), velocityFRD(:, 3)));
    omegaFRD = deg2rad([one.wY, one.wX, -one.wZ]);
    specificForceFRD = gravityMps2 * ...
        [one.Acc_Y, one.Acc_X, -one.Acc_Z];

    alphaDot = zeros(sampleCount, 1);
    betaDot = zeros(sampleCount, 1);
    filteredAlphaDot = 0;
    filteredBetaDot = 0;
    for sample = 2:sampleCount
        rawAlphaDot = max(-10, min(10, ...
            (alphaRad(sample) - alphaRad(sample-1)) / sampleTimeS));
        rawBetaDot = max(-10, min(10, ...
            (betaRad(sample) - betaRad(sample-1)) / sampleTimeS));
        filterGain = sampleTimeS / (0.05 + sampleTimeS);
        filteredAlphaDot = filteredAlphaDot + ...
            filterGain * (rawAlphaDot - filteredAlphaDot);
        filteredBetaDot = filteredBetaDot + ...
            filterGain * (rawBetaDot - filteredBetaDot);
        alphaDot(sample) = filteredAlphaDot;
        betaDot(sample) = filteredBetaDot;
    end

    densityISA = 1.225 * max(0.1, 1 - 2.25577e-5 * ...
        max(-500, min(11000, one.NavAltitude))).^4.25588;
    mappedThrottle = min(1, max(0, 1.25 * one.FW_Thr / 100));

    flightCases(caseNumber).Source = "xiangyi";
    flightCases(caseNumber).Name = xiangyiNames(flightNumber);
    flightCases(caseNumber).TimeS = timeS;
    flightCases(caseNumber).LatitudeDeg = one.Lat;
    flightCases(caseNumber).LongitudeDeg = one.Lon;
    flightCases(caseNumber).HeightAGLM = one.NavHeight;
    flightCases(caseNumber).AltitudeMSLM = one.NavAltitude;
    flightCases(caseNumber).VelocityNEDMps = velocityNED;
    flightCases(caseNumber).VelocityFRDMps = velocityFRD;
    flightCases(caseNumber).TASMps = one.TAS;
    flightCases(caseNumber).YawDeg = one.Yaw;
    flightCases(caseNumber).PitchDeg = one.Pitch;
    flightCases(caseNumber).RollDeg = one.Roll;
    flightCases(caseNumber).VerticalSpeedMps = one.Vz;
    flightCases(caseNumber).OmegaFRDRadS = omegaFRD;
    flightCases(caseNumber).SpecificForceFRDMps2 = specificForceFRD;
    flightCases(caseNumber).GroundTruthSpecificForceFRDMps2 = ...
        nan(sampleCount, 3);
    flightCases(caseNumber).AlphaDeg = rad2deg(alphaRad);
    flightCases(caseNumber).BetaDeg = rad2deg(betaRad);
    flightCases(caseNumber).AlphaDotRadS = alphaDot;
    flightCases(caseNumber).BetaDotRadS = betaDot;
    flightCases(caseNumber).DensityISAKgM3 = densityISA;
    flightCases(caseNumber).DensityLoggedKgM3 = nan(sampleCount, 1);
    flightCases(caseNumber).DeltaDocDeg = ...
        [one.FW_Ail, one.FW_Ele, one.FW_Rud, one.Canard];
    flightCases(caseNumber).ThetaJointDeg = nan(sampleCount, 8);
    flightCases(caseNumber).ThrottleTarget = max(0, min(1, one.FW_Thr / 100));
    flightCases(caseNumber).ThrottleState = mappedThrottle;
    flightCases(caseNumber).PluginThrustN = nan(sampleCount, 1);
    flightCases(caseNumber).PluginTorqueNm = nan(sampleCount, 1);
    flightCases(caseNumber).PluginCoefficient = nan(sampleCount, 6);
    flightCases(caseNumber).Landed = one.NavHeight < 0.5;
    flightCases(caseNumber).MissionSeq = nan(sampleCount, 1);
    flightCases(caseNumber).AccelBiasFRDMps2 = zeros(sampleCount, 3);
end

%% 2.2 PX4 ULog：诊断真值、实际舵角、估计器偏置和地面真值
wantedTopics = [ ...
    "vehicle_global_position", "vehicle_local_position_groundtruth", ...
    "vehicle_attitude", "vehicle_angular_velocity", ...
    "vehicle_acceleration", "estimator_sensor_bias", ...
    "vehicle_land_detected", "mission_result", "vehicle_air_data", ...
    "honghu_v8_aero_state", "honghu_v8_propulsion_state"];

for repeatNumber = 1:numel(px4UlogFiles)
    caseNumber = caseNumber + 1;
    disp("读取PX4 ULog：" + px4UlogFiles(repeatNumber))
    ulogObject = ulogreader(px4UlogFiles(repeatNumber));
    topicRows = readTopicMsgs(ulogObject, "TopicNames", wantedTopics);
    topicNames = string(topicRows.TopicNames);
    assert(all(ismember(wantedTopics, topicNames)), ...
        "PX4 ULog缺少统一气动核对所需主题。");

    globalMessages = topicRows.TopicMessages{find( ...
        topicNames == "vehicle_global_position", 1)};
    localMessages = topicRows.TopicMessages{find( ...
        topicNames == "vehicle_local_position_groundtruth", 1)};
    attitudeMessages = topicRows.TopicMessages{find( ...
        topicNames == "vehicle_attitude", 1)};
    angularMessages = topicRows.TopicMessages{find( ...
        topicNames == "vehicle_angular_velocity", 1)};
    accelerationMessages = topicRows.TopicMessages{find( ...
        topicNames == "vehicle_acceleration", 1)};
    biasMessages = topicRows.TopicMessages{find( ...
        topicNames == "estimator_sensor_bias", 1)};
    landMessages = topicRows.TopicMessages{find( ...
        topicNames == "vehicle_land_detected", 1)};
    missionMessages = topicRows.TopicMessages{find( ...
        topicNames == "mission_result", 1)};
    airMessages = topicRows.TopicMessages{find( ...
        topicNames == "vehicle_air_data", 1)};
    aeroMessages = topicRows.TopicMessages{find( ...
        topicNames == "honghu_v8_aero_state", 1)};
    propulsionMessages = topicRows.TopicMessages{find( ...
        topicNames == "honghu_v8_propulsion_state", 1)};

    tGlobal = seconds(globalMessages.timestamp);
    tLocal = seconds(localMessages.timestamp);
    tAttitude = seconds(attitudeMessages.timestamp);
    tAngular = seconds(angularMessages.timestamp);
    tAcceleration = seconds(accelerationMessages.timestamp);
    tBias = seconds(biasMessages.timestamp);
    tLand = seconds(landMessages.timestamp);
    tMission = seconds(missionMessages.timestamp);
    tAir = seconds(airMessages.timestamp);
    tAero = seconds(aeroMessages.timestamp);
    tPropulsion = seconds(propulsionMessages.timestamp);

    % ULog中少数主题可能在同一timestamp发布两次。interp1要求时间基准
    % 严格唯一，因此对每个主题保留第一次到达的样本。
    [tGlobal, uniqueIndex] = unique(tGlobal, "stable");
    globalMessages = globalMessages(uniqueIndex, :);
    [tLocal, uniqueIndex] = unique(tLocal, "stable");
    localMessages = localMessages(uniqueIndex, :);
    [tAttitude, uniqueIndex] = unique(tAttitude, "stable");
    attitudeMessages = attitudeMessages(uniqueIndex, :);
    [tAngular, uniqueIndex] = unique(tAngular, "stable");
    angularMessages = angularMessages(uniqueIndex, :);
    [tAcceleration, uniqueIndex] = unique(tAcceleration, "stable");
    accelerationMessages = accelerationMessages(uniqueIndex, :);
    [tBias, uniqueIndex] = unique(tBias, "stable");
    biasMessages = biasMessages(uniqueIndex, :);
    [tLand, uniqueIndex] = unique(tLand, "stable");
    landMessages = landMessages(uniqueIndex, :);
    [tMission, uniqueIndex] = unique(tMission, "stable");
    missionMessages = missionMessages(uniqueIndex, :);
    [tAir, uniqueIndex] = unique(tAir, "stable");
    airMessages = airMessages(uniqueIndex, :);
    [tAero, uniqueIndex] = unique(tAero, "stable");
    aeroMessages = aeroMessages(uniqueIndex, :);
    [tPropulsion, uniqueIndex] = unique(tPropulsion, "stable");
    propulsionMessages = propulsionMessages(uniqueIndex, :);

    commonStart = max([tGlobal(1), tLocal(1), tAttitude(1), ...
        tAngular(1), tAcceleration(1), tBias(1), tAir(1), ...
        tAero(1), tPropulsion(1)]);
    % mission_result只在任务状态变化时更新，最后一条消息早于真实接地；
    % 若把它纳入共同结束时间，会把日志截断在约656 s并丢失整段进近、
    % 接地和鸭翼刹车。land/misson/bias均使用previous+extrap，时间边界
    % 只由持续发布的动力学与诊断主题决定。
    commonEnd = min([tGlobal(end), tLocal(end), tAttitude(end), ...
        tAngular(end), tAcceleration(end), tAir(end), ...
        tAero(end), tPropulsion(end)]);
    targetTime = (ceil(commonStart / sampleTimeS) * sampleTimeS: ...
        sampleTimeS:floor(commonEnd / sampleTimeS) * sampleTimeS)';
    assert(numel(targetTime) > 100, "PX4 ULog共同时间区间不足。");

    latitudeDeg = interp1(tGlobal, globalMessages.lat, ...
        targetTime, "linear");
    longitudeDeg = interp1(tGlobal, globalMessages.lon, ...
        targetTime, "linear");
    altitudeMSLM = interp1(tGlobal, globalMessages.alt, ...
        targetTime, "linear");
    positionNED = interp1(tLocal, [localMessages.x, localMessages.y, ...
        localMessages.z], targetTime, "linear");
    velocityNED = interp1(tLocal, [localMessages.vx, localMessages.vy, ...
        localMessages.vz], targetTime, "linear");
    accelerationNED = interp1(tLocal, [localMessages.ax, localMessages.ay, ...
        localMessages.az], targetTime, "linear");

    rawQuaternion = attitudeMessages.q;
    for sample = 2:size(rawQuaternion, 1)
        if dot(rawQuaternion(sample-1, :), rawQuaternion(sample, :)) < 0
            rawQuaternion(sample, :) = -rawQuaternion(sample, :);
        end
    end
    quaternion = interp1(tAttitude, rawQuaternion, targetTime, "linear");
    quaternion = quaternion ./ vecnorm(quaternion, 2, 2);
    qw = quaternion(:, 1); qx = quaternion(:, 2);
    qy = quaternion(:, 3); qz = quaternion(:, 4);

    rollRad = atan2(2 * (qw .* qx + qy .* qz), ...
        1 - 2 * (qx.^2 + qy.^2));
    pitchRad = asin(max(-1, min(1, 2 * (qw .* qy - qz .* qx))));
    yawRad = unwrap(atan2(2 * (qw .* qz + qx .* qy), ...
        1 - 2 * (qy.^2 + qz.^2)));
    cRoll = cos(rollRad); sRoll = sin(rollRad);
    cPitch = cos(pitchRad); sPitch = sin(pitchRad);
    cYaw = cos(yawRad); sYaw = sin(yawRad);

    velocityFRD = zeros(numel(targetTime), 3);
    velocityFRD(:, 1) = cPitch .* cYaw .* velocityNED(:, 1) + ...
        cPitch .* sYaw .* velocityNED(:, 2) - ...
        sPitch .* velocityNED(:, 3);
    velocityFRD(:, 2) = ...
        (sRoll .* sPitch .* cYaw - cRoll .* sYaw) .* velocityNED(:, 1) + ...
        (sRoll .* sPitch .* sYaw + cRoll .* cYaw) .* velocityNED(:, 2) + ...
        sRoll .* cPitch .* velocityNED(:, 3);
    velocityFRD(:, 3) = ...
        (cRoll .* sPitch .* cYaw + sRoll .* sYaw) .* velocityNED(:, 1) + ...
        (cRoll .* sPitch .* sYaw - sRoll .* cYaw) .* velocityNED(:, 2) + ...
        cRoll .* cPitch .* velocityNED(:, 3);

    accelerationMinusGravity = accelerationNED - ...
        [zeros(numel(targetTime), 2), gravityMps2 * ones(numel(targetTime), 1)];
    groundTruthSpecificForceFRD = zeros(numel(targetTime), 3);
    groundTruthSpecificForceFRD(:, 1) = ...
        cPitch .* cYaw .* accelerationMinusGravity(:, 1) + ...
        cPitch .* sYaw .* accelerationMinusGravity(:, 2) - ...
        sPitch .* accelerationMinusGravity(:, 3);
    groundTruthSpecificForceFRD(:, 2) = ...
        (sRoll .* sPitch .* cYaw - cRoll .* sYaw) .* accelerationMinusGravity(:, 1) + ...
        (sRoll .* sPitch .* sYaw + cRoll .* cYaw) .* accelerationMinusGravity(:, 2) + ...
        sRoll .* cPitch .* accelerationMinusGravity(:, 3);
    groundTruthSpecificForceFRD(:, 3) = ...
        (cRoll .* sPitch .* cYaw + sRoll .* sYaw) .* accelerationMinusGravity(:, 1) + ...
        (cRoll .* sPitch .* sYaw - sRoll .* cYaw) .* accelerationMinusGravity(:, 2) + ...
        cRoll .* cPitch .* accelerationMinusGravity(:, 3);

    omegaFRDEstimator = interp1(tAngular, angularMessages.xyz, ...
        targetTime, "linear");
    vehicleAcceleration = interp1(tAcceleration, ...
        accelerationMessages.xyz, targetTime, "linear");
    accelBias = interp1(tBias, biasMessages.accel_bias, ...
        targetTime, "previous", "extrap");
    accelBiasValid = interp1(tBias, double(biasMessages.accel_bias_valid), ...
        targetTime, "previous", "extrap") > 0.5;
    specificForceFRD = vehicleAcceleration + accelBias;
    specificForceFRD(~accelBiasValid, :) = NaN;

    aeroState = interp1(tAero, [ ...
        aeroMessages.airspeed_m_s, aeroMessages.alpha_deg, ...
        aeroMessages.beta_deg, aeroMessages.rho_kg_m3, ...
        aeroMessages.alpha_dot_rad_s, aeroMessages.beta_dot_rad_s, ...
        aeroMessages.body_rates_frd_rad_s, aeroMessages.coefficients, ...
        aeroMessages.joint_angles_deg, aeroMessages.delta_doc_deg], ...
        targetTime, "linear");
    thetaJointDeg = aeroState(:, 16:23);
    deltaDocDeg = [ ...
        0.5 * (-thetaJointDeg(:, 1) + thetaJointDeg(:, 2)), ...
        0.5 * ( thetaJointDeg(:, 3) + thetaJointDeg(:, 4)), ...
        0.5 * ( thetaJointDeg(:, 5) + thetaJointDeg(:, 6)), ...
        0.5 * ( thetaJointDeg(:, 7) + thetaJointDeg(:, 8))];

    propulsionState = interp1(tPropulsion, [ ...
        propulsionMessages.target_throttle, ...
        propulsionMessages.filtered_throttle, ...
        propulsionMessages.thrust_n, propulsionMessages.torque_nm], ...
        targetTime, "linear");
    densityLogged = interp1(tAir, airMessages.rho, targetTime, "linear");
    landed = interp1(tLand, double(landMessages.landed), ...
        targetTime, "previous", "extrap") > 0.5;
    missionSeq = interp1(tMission, double(missionMessages.seq_current), ...
        targetTime, "previous", "extrap");

    % PX4/Gazebo的正向链必须使用插件实际采用的相对气流、FRD角速度
    % 和50 Hz内部滤波后的alpha_dot/beta_dot。若由20 Hz导航真值重新
    % 求导，会把时间离散误差误认为气动表误差，无法满足第三条插件
    % 真值校验链。导航速度重建值仍保留在VelocityFRDMps中用于独立
    % 坐标和动力学交叉检查。
    tasMps = aeroState(:, 1);
    alphaRad = deg2rad(aeroState(:, 2));
    betaRad = deg2rad(aeroState(:, 3));
    alphaDot = aeroState(:, 5);
    betaDot = aeroState(:, 6);
    omegaFRD = aeroState(:, 7:9);

    heightAGLM = -positionNED(:, 3) - groundBaseZM;
    densityISA = 1.225 * max(0.1, 1 - 2.25577e-5 * ...
        max(-500, min(11000, altitudeMSLM))).^4.25588;

    flightCases(caseNumber).Source = "px4";
    [~, ulogBaseName] = fileparts(px4UlogFiles(repeatNumber));
    flightCases(caseNumber).Name = "px4_" + string(ulogBaseName);
    flightCases(caseNumber).TimeS = targetTime - targetTime(1);
    flightCases(caseNumber).LatitudeDeg = latitudeDeg;
    flightCases(caseNumber).LongitudeDeg = longitudeDeg;
    flightCases(caseNumber).HeightAGLM = heightAGLM;
    flightCases(caseNumber).AltitudeMSLM = altitudeMSLM;
    flightCases(caseNumber).VelocityNEDMps = velocityNED;
    flightCases(caseNumber).VelocityFRDMps = velocityFRD;
    flightCases(caseNumber).TASMps = tasMps;
    flightCases(caseNumber).YawDeg = mod(rad2deg(yawRad), 360);
    flightCases(caseNumber).PitchDeg = rad2deg(pitchRad);
    flightCases(caseNumber).RollDeg = rad2deg(rollRad);
    flightCases(caseNumber).VerticalSpeedMps = -velocityNED(:, 3);
    flightCases(caseNumber).OmegaFRDRadS = omegaFRD;
    flightCases(caseNumber).EstimatorOmegaFRDRadS = omegaFRDEstimator;
    flightCases(caseNumber).SpecificForceFRDMps2 = specificForceFRD;
    flightCases(caseNumber).GroundTruthSpecificForceFRDMps2 = ...
        groundTruthSpecificForceFRD;
    flightCases(caseNumber).AlphaDeg = rad2deg(alphaRad);
    flightCases(caseNumber).BetaDeg = rad2deg(betaRad);
    flightCases(caseNumber).AlphaDotRadS = alphaDot;
    flightCases(caseNumber).BetaDotRadS = betaDot;
    flightCases(caseNumber).DensityISAKgM3 = densityISA;
    flightCases(caseNumber).DensityLoggedKgM3 = densityLogged;
    flightCases(caseNumber).DeltaDocDeg = deltaDocDeg;
    flightCases(caseNumber).ThetaJointDeg = thetaJointDeg;
    flightCases(caseNumber).ThrottleTarget = propulsionState(:, 1);
    flightCases(caseNumber).ThrottleState = propulsionState(:, 2);
    flightCases(caseNumber).PluginThrustN = propulsionState(:, 3);
    flightCases(caseNumber).PluginTorqueNm = propulsionState(:, 4);
    flightCases(caseNumber).PluginCoefficient = aeroState(:, 10:15);
    flightCases(caseNumber).Landed = landed;
    flightCases(caseNumber).MissionSeq = missionSeq;
    flightCases(caseNumber).AccelBiasFRDMps2 = accelBias;
end

%% 2.3 统一坐标和数据质量自检
frameChecks = table;
for checkCaseNumber = 1:numel(flightCases)
    one = flightCases(checkCaseNumber);
    speedDifference = vecnorm(one.VelocityFRDMps, 2, 2) - one.TASMps;
    speedRMSE = sqrt(mean(speedDifference.^2, "omitnan"));

    if one.Source == "px4"
        forceResidual = one.SpecificForceFRDMps2 - ...
            one.GroundTruthSpecificForceFRDMps2;
        % 地面碰撞脉冲并非气动反算样本，而且估计器与Gazebo真值在
        % 冲击瞬间带宽不同。坐标/比力自检只在共同有效空中包线内做。
        forceCheckMask = one.HeightAGLM >= 5 & one.TASMps >= 20 & ...
            all(isfinite(forceResidual), 2);
        forceRMSE = sqrt(mean(forceResidual(forceCheckMask, :).^2, ...
            1, "omitnan"));
    else
        % 翔仪用导航速度导数重建比力，仅作为坐标自洽检查。
        windowSamples = 21;
        windowIndex = (0:windowSamples-1)';
        checkWindow = 0.5 - 0.5 * cos( ...
            2 * pi * windowIndex / (windowSamples - 1));
        checkWindow = checkWindow / sum(checkWindow);
        accelNED = zeros(size(one.VelocityNEDMps));
        for axisNumber = 1:3
            smoothVelocity = conv(one.VelocityNEDMps(:, axisNumber), ...
                checkWindow, "same");
            accelNED(:, axisNumber) = gradient(smoothVelocity, sampleTimeS);
        end
        forceRMSE = [NaN, NaN, NaN];
    end

    frameChecks = [frameChecks; table( ...
        one.Source, one.Name, numel(one.TimeS), speedRMSE, ...
        forceRMSE(1), forceRMSE(2), forceRMSE(3), ...
        VariableNames=["Source", "Flight", "Samples", "TasBodySpeedRMSE", ...
        "SpecificForceRMSEX", "SpecificForceRMSEY", ...
        "SpecificForceRMSEZ"])]; %#ok<AGROW>
end

assert(all(frameChecks.TasBodySpeedRMSE < 0.15), ...
    "至少一个架次的TAS/机体系速度模自检未通过。");
disp("统一日志导入完成：")
disp(frameChecks)
