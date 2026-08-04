%% 3. 对全部架次使用同一套六自由度反算和V8正向气动模型

caseResults = struct([]);
coefficientMetrics = table;
pluginMetrics = table;
engineMetrics = table;
sensitivityMetrics = table;

for caseIndex = 1:numel(flightCases)
    one = flightCases(caseIndex);
    one.TASMps = double(one.TASMps);
    one.AltitudeMSLM = double(one.AltitudeMSLM);
    one.HeightAGLM = double(one.HeightAGLM);
    one.RollDeg = double(one.RollDeg);
    one.VerticalSpeedMps = double(one.VerticalSpeedMps);
    one.AlphaDotRadS = double(one.AlphaDotRadS);
    one.BetaDotRadS = double(one.BetaDotRadS);
    one.SpecificForceFRDMps2 = double(one.SpecificForceFRDMps2);
    one.DensityISAKgM3 = double(one.DensityISAKgM3);
    one.DensityLoggedKgM3 = double(one.DensityLoggedKgM3);
    one.ThrottleState = double(one.ThrottleState);
    one.PluginThrustN = double(one.PluginThrustN);
    one.PluginTorqueNm = double(one.PluginTorqueNm);
    one.PluginCoefficient = double(one.PluginCoefficient);
    sampleCount = numel(one.TimeS);
    % ulogreader保留部分uORB字段的single类型，而CSV和查表轴为double。
    % interp2要求两个查询坐标类型一致，因此计算入口统一转换为double。
    alphaDeg = double(one.AlphaDeg);
    betaDeg = double(one.BetaDeg);
    alphaRad = deg2rad(alphaDeg);
    betaRad = deg2rad(betaDeg);
    omegaFRD = double(one.OmegaFRDRadS);
    deltaA = double(one.DeltaDocDeg(:, 1));
    deltaE = double(one.DeltaDocDeg(:, 2));
    deltaR = double(one.DeltaDocDeg(:, 3));
    deltaC = double(one.DeltaDocDeg(:, 4));

    disp("------------------------------------------------------------")
    disp("统一气动计算：" + one.Name)

    %% 3.1 静态六分量气动表
    staticCoefficient = zeros(sampleCount, 6);
    absoluteBeta = min(abs(betaDeg), 16);
    lateralSign = ones(sampleCount, 1);
    lateralSign(betaDeg < 0) = -1;

    for coefficientNumber = 1:6
        coefficientName = coefficientNames(coefficientNumber);
        rowAxis = staticGrid.(coefficientName).alpha;
        columnAxis = staticGrid.(coefficientName).beta;
        valueGrid = staticGrid.(coefficientName).value;
        queryAlpha = max(rowAxis(1), min(rowAxis(end), alphaDeg));
        queryBeta = max(columnAxis(1), min(columnAxis(end), absoluteBeta));
        coefficientValue = interp2(columnAxis, rowAxis, valueGrid, ...
            queryBeta, queryAlpha, "linear");

        lowAlphaMask = alphaDeg < rowAxis(1);
        if any(lowAlphaMask)
            firstValue = interp2(columnAxis, rowAxis, valueGrid, ...
                queryBeta(lowAlphaMask), ...
                rowAxis(1) * ones(nnz(lowAlphaMask), 1), "linear");
            secondValue = interp2(columnAxis, rowAxis, valueGrid, ...
                queryBeta(lowAlphaMask), ...
                rowAxis(2) * ones(nnz(lowAlphaMask), 1), "linear");
            slopeValue = (secondValue - firstValue) / ...
                (rowAxis(2) - rowAxis(1));
            coefficientValue(lowAlphaMask) = firstValue + slopeValue .* ...
                (alphaDeg(lowAlphaMask) - rowAxis(1));
        end

        if any(coefficientName == ["CY", "Cl", "Cn"])
            coefficientValue = coefficientValue .* lateralSign;
        end
        staticCoefficient(:, coefficientNumber) = coefficientValue;
    end

    %% 3.2 副翼、升降舵、方向舵和鸭翼贡献
    controlContribution = zeros(sampleCount, 6);

    deltaALookup = max(-10, min(10, deltaA));
    fadeInput = max(0, min(1, (abs(alphaDeg) - 12) / 8));
    aileronFade = 1 - fadeInput.^2 .* (3 - 2 * fadeInput);
    aileronTargets = ["CD", "CY", "Cl", "Cn"];
    aileronColumns = [2, 3, 4, 6];
    for targetNumber = 1:numel(aileronTargets)
        gridName = "aileron_" + aileronTargets(targetNumber);
        rowAxis = controlGrid.(gridName).alpha;
        columnAxis = controlGrid.(gridName).column;
        valueGrid = controlGrid.(gridName).value;
        queryAlpha = max(rowAxis(1), min(rowAxis(end), alphaDeg));
        derivative = interp2(columnAxis, rowAxis, valueGrid, ...
            deltaALookup, queryAlpha, "linear");
        controlContribution(:, aileronColumns(targetNumber)) = ...
            controlContribution(:, aileronColumns(targetNumber)) + ...
            derivative .* deltaA .* aileronFade;
    end

    deltaELookup = max(-10, min(20, deltaE));
    elevatorTargets = ["CL", "CD", "Cm"];
    elevatorColumns = [1, 2, 5];
    for targetNumber = 1:numel(elevatorTargets)
        gridName = "elevator_" + elevatorTargets(targetNumber);
        rowAxis = controlGrid.(gridName).alpha;
        columnAxis = controlGrid.(gridName).column;
        valueGrid = controlGrid.(gridName).value;
        queryAlpha = max(rowAxis(1), min(rowAxis(end), alphaDeg));
        derivative = interp2(columnAxis, rowAxis, valueGrid, ...
            deltaELookup, queryAlpha, "linear");
        controlContribution(:, elevatorColumns(targetNumber)) = ...
            controlContribution(:, elevatorColumns(targetNumber)) + ...
            derivative .* deltaE;
    end

    reflectedBeta = betaDeg;
    reflectedBeta(deltaR < 0) = -reflectedBeta(deltaR < 0);
    rudderTargets = ["CD", "CY", "Cl", "Cn"];
    rudderColumns = [2, 3, 4, 6];
    for targetNumber = 1:numel(rudderTargets)
        gridName = "rudder_" + rudderTargets(targetNumber);
        rowAxis = controlGrid.(gridName).alpha;
        columnAxis = controlGrid.(gridName).column;
        valueGrid = controlGrid.(gridName).value;
        queryAlpha = max(rowAxis(1), min(rowAxis(end), alphaDeg));
        queryBeta = max(columnAxis(1), min(columnAxis(end), reflectedBeta));
        derivative = interp2(columnAxis, rowAxis, valueGrid, ...
            queryBeta, queryAlpha, "linear");
        if rudderTargets(targetNumber) == "CD"
            rudderCdSign = ones(sampleCount, 1);
            rudderCdSign(deltaR < 0) = -1;
            derivative = derivative .* rudderCdSign;
        end
        controlContribution(:, rudderColumns(targetNumber)) = ...
            controlContribution(:, rudderColumns(targetNumber)) + ...
            derivative .* deltaR;
    end

    deltaCEffective = max(-4, min(15, deltaC));
    deltaCEffective(deltaC < -4) = -4;
    deltaCLookup = max(-4, min(8, deltaCEffective));
    fadeInput = max(0, min(1, ...
        (abs(alphaDeg + deltaCEffective) - 12) / 4));
    canardFade = 1 - fadeInput.^2 .* (3 - 2 * fadeInput);
    canardTargets = ["CL", "CD", "Cm"];
    canardColumns = [1, 2, 5];
    for targetNumber = 1:numel(canardTargets)
        gridName = "canard_" + canardTargets(targetNumber);
        rowAxis = controlGrid.(gridName).alpha;
        columnAxis = controlGrid.(gridName).column;
        valueGrid = controlGrid.(gridName).value;
        queryAlpha = max(rowAxis(1), min(rowAxis(end), alphaDeg));
        derivative = interp2(columnAxis, rowAxis, valueGrid, ...
            deltaCLookup, queryAlpha, "linear");
        controlContribution(:, canardColumns(targetNumber)) = ...
            controlContribution(:, canardColumns(targetNumber)) + ...
            derivative .* deltaCEffective .* canardFade;
    end

    %% 3.3 动导数
    modelCoefficientRaw = staticCoefficient + controlContribution;
    speedForDerivative = max(one.TASMps, 5);
    rateBlendInput = max(0, min(1, (one.TASMps - 3) / 2));
    rateBlend = rateBlendInput.^2 .* (3 - 2 * rateBlendInput);
    inverseTwoV = 0.5 ./ speedForDerivative;
    pRate = omegaFRD(:, 1);
    qRate = omegaFRD(:, 2);
    rRate = omegaFRD(:, 3);

    modelCoefficientRaw(:, 1) = modelCoefficientRaw(:, 1) + ...
        rateBlend .* 5.62 .* qRate .* macM .* inverseTwoV;
    modelCoefficientRaw(:, 3) = modelCoefficientRaw(:, 3) + ...
        rateBlend .* (-0.15 .* pRate .* spanM .* inverseTwoV + ...
        0.34 .* rRate .* spanM .* inverseTwoV);
    modelCoefficientRaw(:, 4) = modelCoefficientRaw(:, 4) + ...
        rateBlend .* (-0.33 .* pRate .* spanM .* inverseTwoV + ...
        0.10 .* rRate .* spanM .* inverseTwoV);
    modelCoefficientRaw(:, 5) = modelCoefficientRaw(:, 5) + ...
        rateBlend .* (-7.00 .* qRate .* macM .* inverseTwoV - ...
        0.33 .* one.AlphaDotRadS .* macM .* inverseTwoV);
    modelCoefficientRaw(:, 6) = modelCoefficientRaw(:, 6) + ...
        rateBlend .* (-0.05 .* pRate .* spanM .* inverseTwoV - ...
        0.08 .* rRate .* spanM .* inverseTwoV + ...
        0.14 .* one.BetaDotRadS .* spanM .* inverseTwoV);

    %% 3.4 使用共同发动机表计算推力、转速和扭矩
    thrustN = zeros(sampleCount, 1);
    torqueNm = zeros(sampleCount, 1);
    rpm = zeros(sampleCount, 1);
    propulsionClamped = false(sampleCount, 1);

    for sample = 1:sampleCount
        altitudeQuery = max(propellerAltitudes(1), ...
            min(propellerAltitudes(end), one.AltitudeMSLM(sample)));
        speedQuery = max(0, min(50, one.TASMps(sample)));
        throttleQuery = max(0, min(100, 100 * one.ThrottleState(sample)));
        propulsionClamped(sample) = ...
            one.AltitudeMSLM(sample) < propellerAltitudes(1) || ...
            one.AltitudeMSLM(sample) > propellerAltitudes(end) || ...
            one.TASMps(sample) < 0 || one.TASMps(sample) > 50;

        altitudeHigh = find(propellerAltitudes > altitudeQuery, 1, "first");
        if isempty(altitudeHigh)
            altitudeHigh = numel(propellerAltitudes);
        end
        altitudeHigh = max(2, altitudeHigh);
        altitudeLow = altitudeHigh - 1;
        altitudePair = propellerAltitudes([altitudeLow, altitudeHigh]);
        resultAtAltitude = zeros(2, 3);

        for altitudeSlot = 1:2
            altitudeValue = altitudePair(altitudeSlot);
            atAltitude = propellerTable( ...
                propellerTable.altitude_m == altitudeValue, :);
            throttleLevels = [0; unique(atAltitude.throttle_pct)];
            throttleAtLevel = max(0, min(throttleLevels(end), throttleQuery));
            throttleHigh = find(throttleLevels > throttleAtLevel, 1, "first");
            if isempty(throttleHigh)
                throttleHigh = numel(throttleLevels);
            end
            throttleHigh = max(2, throttleHigh);
            throttleLow = throttleHigh - 1;
            throttlePair = throttleLevels([throttleLow, throttleHigh]);
            resultAtThrottle = zeros(2, 3);

            for throttleSlot = 1:2
                throttleValue = throttlePair(throttleSlot);
                if throttleValue == 0
                    resultAtThrottle(throttleSlot, :) = [0, 0, 0];
                else
                    tableRows = atAltitude( ...
                        atAltitude.throttle_pct == throttleValue, :);
                    tableSpeed = tableRows.airspeed_mps;
                    speedAtLevel = max(tableSpeed(1), ...
                        min(tableSpeed(end), speedQuery));
                    resultAtThrottle(throttleSlot, 1) = interp1( ...
                        tableSpeed, tableRows.thrust_kgf * 9.80665, ...
                        speedAtLevel, "linear");
                    resultAtThrottle(throttleSlot, 2) = interp1( ...
                        tableSpeed, tableRows.torque_Nm, ...
                        speedAtLevel, "linear");
                    resultAtThrottle(throttleSlot, 3) = tableRows.rpm(1);
                end
            end

            throttleFraction = (throttleAtLevel - throttlePair(1)) / ...
                (throttlePair(2) - throttlePair(1));
            resultAtAltitude(altitudeSlot, :) = ...
                resultAtThrottle(1, :) + throttleFraction * ...
                (resultAtThrottle(2, :) - resultAtThrottle(1, :));
        end

        altitudeFraction = (altitudeQuery - altitudePair(1)) / ...
            (altitudePair(2) - altitudePair(1));
        engineResult = resultAtAltitude(1, :) + altitudeFraction * ...
            (resultAtAltitude(2, :) - resultAtAltitude(1, :));
        thrustN(sample) = engineResult(1);
        torqueNm(sample) = engineResult(2);
        rpm(sample) = engineResult(3);
    end

    %% 3.5 发动机力矩和刚体动力学反算
    thrustDirectionFRD = [cos(thrustDownRad), 0, sin(thrustDownRad)];
    propellerForceFRD = thrustN .* thrustDirectionFRD;
    reactionMomentFRD = reactionTorqueSignFRD * ...
        [torqueNm, zeros(sampleCount, 2)];
    propellerMomentFRD = cross( ...
        repmat(enginePointFRDM, sampleCount, 1), ...
        propellerForceFRD, 2) + reactionMomentFRD;

    filterSamples = round(0.5 / sampleTimeS);
    if mod(filterSamples, 2) == 0
        filterSamples = filterSamples + 1;
    end
    hannIndex = (0:filterSamples-1)';
    hannWindow = 0.5 - 0.5 * cos( ...
        2 * pi * hannIndex / (filterSamples - 1));
    hannWindow = hannWindow / sum(hannWindow);

    omegaFiltered = zeros(size(omegaFRD));
    omegaDot = zeros(size(omegaFRD));
    for axisNumber = 1:3
        omegaFiltered(:, axisNumber) = conv( ...
            omegaFRD(:, axisNumber), hannWindow, "same");
        omegaDot(:, axisNumber) = gradient( ...
            omegaFiltered(:, axisNumber), sampleTimeS);
    end

    aerodynamicForceFRD = massKg * one.SpecificForceFRDMps2 - ...
        propellerForceFRD;
    inertiaOmega = omegaFiltered * inertiaFRD';
    totalMomentFRD = omegaDot * inertiaFRD' + ...
        cross(omegaFiltered, inertiaOmega, 2);
    aerodynamicMomentFRD = totalMomentFRD - propellerMomentFRD;

    density = one.DensityISAKgM3;
    dynamicPressure = 0.5 * density .* one.TASMps.^2;
    forceScale = dynamicPressure * areaM2;
    cosAlpha = cos(alphaRad); sinAlpha = sin(alphaRad);
    cosBeta = cos(betaRad); sinBeta = sin(betaRad);
    windX = [cosAlpha .* cosBeta, sinBeta, sinAlpha .* cosBeta];
    windY = [-cosAlpha .* sinBeta, cosBeta, -sinAlpha .* sinBeta];
    windZ = [-sinAlpha, zeros(sampleCount, 1), cosAlpha];

    inverseCoefficientRaw = zeros(sampleCount, 6);
    inverseCoefficientRaw(:, 1) = -sum( ...
        aerodynamicForceFRD .* windZ, 2) ./ forceScale;
    inverseCoefficientRaw(:, 2) = -sum( ...
        aerodynamicForceFRD .* windX, 2) ./ forceScale;
    inverseCoefficientRaw(:, 3) = sum( ...
        aerodynamicForceFRD .* windY, 2) ./ forceScale;
    inverseCoefficientRaw(:, 4) = aerodynamicMomentFRD(:, 1) ./ ...
        (forceScale * spanM);
    inverseCoefficientRaw(:, 5) = aerodynamicMomentFRD(:, 2) ./ ...
        (forceScale * macM);
    inverseCoefficientRaw(:, 6) = aerodynamicMomentFRD(:, 3) ./ ...
        (forceScale * spanM);

    %% 3.6 主有效样本、工况分类和0.5 s平滑
    halfWindow = floor(filterSamples / 2);
    edgeMask = false(sampleCount, 1);
    edgeMask(halfWindow+1:end-halfWindow) = true;
    valid = one.TASMps >= 20 & one.HeightAGLM >= 5 & ...
        dynamicPressure >= 200 & ~one.Landed & ~propulsionClamped & ...
        alphaDeg >= -2 & alphaDeg <= 20 & abs(betaDeg) <= 16 & ...
        edgeMask & all(isfinite(one.SpecificForceFRDMps2), 2) & ...
        all(isfinite(inverseCoefficientRaw), 2) & ...
        all(isfinite(modelCoefficientRaw), 2);

    inverseCoefficient = zeros(size(inverseCoefficientRaw));
    modelCoefficient = zeros(size(modelCoefficientRaw));
    pluginCoefficient = zeros(size(one.PluginCoefficient));
    for coefficientNumber = 1:6
        inverseCoefficient(:, coefficientNumber) = conv( ...
            inverseCoefficientRaw(:, coefficientNumber), ...
            hannWindow, "same");
        modelCoefficient(:, coefficientNumber) = conv( ...
            modelCoefficientRaw(:, coefficientNumber), ...
            hannWindow, "same");
        pluginCoefficient(:, coefficientNumber) = conv( ...
            one.PluginCoefficient(:, coefficientNumber), ...
            hannWindow, "same");
    end

    phase = repmat("level", sampleCount, 1);
    phase(one.VerticalSpeedMps > 0.5) = "climb";
    phase(one.VerticalSpeedMps < -0.5) = "descent";
    phase(abs(one.RollDeg) > 10) = "turn";

    %% 3.7 六分量主指标和bootstrap置信区间
    for phaseNumber = 1:numel(phaseNames)
        phaseName = phaseNames(phaseNumber);
        if phaseName == "all"
            phaseMask = valid;
        else
            phaseMask = valid & phase == phaseName;
        end
        if nnz(phaseMask) < 30
            continue
        end

        for coefficientNumber = 1:6
            inverseValue = inverseCoefficient(phaseMask, coefficientNumber);
            modelValue = modelCoefficient(phaseMask, coefficientNumber);
            residual = inverseValue - modelValue;
            biasValue = mean(residual);
            rmseValue = sqrt(mean(residual.^2));
            maeValue = mean(abs(residual));
            correlationMatrix = corrcoef(inverseValue, modelValue);
            correlationValue = correlationMatrix(1, 2);
            if ~isfinite(correlationValue)
                correlationValue = NaN;
            end
            linearFit = polyfit(modelValue, inverseValue, 1);
            normalizationRange = max(inverseValue) - min(inverseValue);
            nrmseValue = rmseValue / max(normalizationRange, 1e-9);

            bootstrapBias = zeros(bootstrapCount, 1);
            bootstrapRMSE = zeros(bootstrapCount, 1);
            bootstrapSampleCount = numel(residual);
            for bootstrapIndex = 1:bootstrapCount
                selected = randi(bootstrapSampleCount, ...
                    bootstrapSampleCount, 1);
                selectedResidual = residual(selected);
                bootstrapBias(bootstrapIndex) = mean(selectedResidual);
                bootstrapRMSE(bootstrapIndex) = ...
                    sqrt(mean(selectedResidual.^2));
            end
            biasCI = prctile(bootstrapBias, [2.5, 97.5]);
            rmseCI = prctile(bootstrapRMSE, [2.5, 97.5]);

            coefficientMetrics = [coefficientMetrics; table( ...
                one.Source, one.Name, phaseName, ...
                coefficientNames(coefficientNumber), nnz(phaseMask), ...
                mean(inverseValue), mean(modelValue), biasValue, ...
                rmseValue, maeValue, correlationValue, linearFit(1), ...
                nrmseValue, biasCI(1), biasCI(2), rmseCI(1), rmseCI(2), ...
                VariableNames=["Source", "Flight", "Phase", ...
                "Coefficient", "Samples", "InverseMean", "ModelMean", ...
                "Bias", "RMSE", "MAE", "Correlation", "Slope", "NRMSE", ...
                "BiasCI95Low", "BiasCI95High", ...
                "RMSECI95Low", "RMSECI95High"])]; %#ok<AGROW>
        end
    end

    %% 3.8 PX4插件系数和推进真值第三链
    if one.Source == "px4"
        for coefficientNumber = 1:6
            pluginMask = valid & isfinite( ...
                pluginCoefficient(:, coefficientNumber));
            difference = modelCoefficient(pluginMask, coefficientNumber) - ...
                pluginCoefficient(pluginMask, coefficientNumber);
            pluginMetrics = [pluginMetrics; table( ...
                one.Name, coefficientNames(coefficientNumber), ...
                nnz(pluginMask), mean(difference), ...
                sqrt(mean(difference.^2)), mean(abs(difference)), ...
                VariableNames=["Flight", "Coefficient", "Samples", ...
                "Bias", "RMSE", "MAE"])]; %#ok<AGROW>
        end
        thrustTruthMask = valid & isfinite(one.PluginThrustN);
        pluginThrustResidual = thrustN(thrustTruthMask) - ...
            one.PluginThrustN(thrustTruthMask);
        pluginTorqueResidual = torqueNm(thrustTruthMask) - ...
            one.PluginTorqueNm(thrustTruthMask);
        pluginThrustRMSE = sqrt(mean(pluginThrustResidual.^2));
        pluginTorqueRMSE = sqrt(mean(pluginTorqueResidual.^2));
    else
        pluginThrustRMSE = NaN;
        pluginTorqueRMSE = NaN;
    end

    %% 3.9 发动机闭合
    aerodynamicForceFromModel = ...
        -modelCoefficientRaw(:, 2) .* forceScale .* windX + ...
         modelCoefficientRaw(:, 3) .* forceScale .* windY - ...
         modelCoefficientRaw(:, 1) .* forceScale .* windZ;
    requiredPropellerForce = massKg * one.SpecificForceFRDMps2 - ...
        aerodynamicForceFromModel;
    requiredThrust = requiredPropellerForce * thrustDirectionFRD';
    aerodynamicMomentFromModel = [ ...
        modelCoefficientRaw(:, 4) .* forceScale * spanM, ...
        modelCoefficientRaw(:, 5) .* forceScale * macM, ...
        modelCoefficientRaw(:, 6) .* forceScale * spanM];
    requiredPropellerMoment = totalMomentFRD - aerodynamicMomentFromModel;
    thrustPointMoment = cross(repmat(enginePointFRDM, sampleCount, 1), ...
        propellerForceFRD, 2);
    requiredReactionMomentX = requiredPropellerMoment(:, 1) - ...
        thrustPointMoment(:, 1);
    cruiseMask = valid & one.TASMps >= 43 & one.TASMps <= 47 & ...
        abs(one.RollDeg) <= 3 & abs(one.VerticalSpeedMps) <= 0.3;
    engineMetrics = [engineMetrics; table( ...
        one.Source, one.Name, nnz(cruiseMask), ...
        mean(requiredThrust(cruiseMask), "omitnan"), ...
        mean(thrustN(cruiseMask), "omitnan"), ...
        sqrt(mean((thrustN(cruiseMask) - ...
        requiredThrust(cruiseMask)).^2, "omitnan")), ...
        mean(requiredReactionMomentX(cruiseMask), "omitnan"), ...
        mean(torqueNm(cruiseMask), "omitnan"), ...
        sqrt(mean((torqueNm(cruiseMask) - ...
        requiredReactionMomentX(cruiseMask)).^2, "omitnan")), ...
        pluginThrustRMSE, pluginTorqueRMSE, ...
        VariableNames=["Source", "Flight", "CruiseSamples", ...
        "RequiredThrustN", "TableThrustN", "ThrustClosureRMSE", ...
        "RequiredReactionMomentXNm", "TableTorqueNm", ...
        "TorqueClosureRMSE", "PluginThrustRMSE", ...
        "PluginTorqueRMSE"])]; %#ok<AGROW>

    %% 3.10 滤波、推力、重力常数和密度口径敏感性
    for sensitivityWindow = filterWindowsS
        sensitivitySamples = round(sensitivityWindow / sampleTimeS);
        if mod(sensitivitySamples, 2) == 0
            sensitivitySamples = sensitivitySamples + 1;
        end
        sensitivityIndex = (0:sensitivitySamples-1)';
        sensitivityWindowVector = 0.5 - 0.5 * cos( ...
            2 * pi * sensitivityIndex / (sensitivitySamples - 1));
        sensitivityWindowVector = sensitivityWindowVector / ...
            sum(sensitivityWindowVector);
        for coefficientNumber = 1:6
            inverseSensitivity = conv( ...
                inverseCoefficientRaw(:, coefficientNumber), ...
                sensitivityWindowVector, "same");
            modelSensitivity = conv( ...
                modelCoefficientRaw(:, coefficientNumber), ...
                sensitivityWindowVector, "same");
            residual = inverseSensitivity(valid) - modelSensitivity(valid);
            sensitivityMetrics = [sensitivityMetrics; table( ...
                one.Source, one.Name, "filter_window", ...
                sensitivityWindow, 1.0, "isa", ...
                coefficientNames(coefficientNumber), ...
                mean(residual), sqrt(mean(residual.^2)), ...
                VariableNames=["Source", "Flight", "Sensitivity", ...
                "WindowS", "ThrustScale", "DensitySource", ...
                "Coefficient", "Bias", "RMSE"])]; %#ok<AGROW>
        end
    end

    for thrustScale = [0.8, 1.2]
        scaledForce = massKg * one.SpecificForceFRDMps2 - ...
            thrustScale * propellerForceFRD;
        scaledMoment = totalMomentFRD - thrustScale * propellerMomentFRD;
        scaledInverse = inverseCoefficientRaw;
        scaledInverse(:, 1) = -sum(scaledForce .* windZ, 2) ./ forceScale;
        scaledInverse(:, 2) = -sum(scaledForce .* windX, 2) ./ forceScale;
        scaledInverse(:, 3) = sum(scaledForce .* windY, 2) ./ forceScale;
        scaledInverse(:, 4) = scaledMoment(:, 1) ./ (forceScale * spanM);
        scaledInverse(:, 5) = scaledMoment(:, 2) ./ (forceScale * macM);
        scaledInverse(:, 6) = scaledMoment(:, 3) ./ (forceScale * spanM);
        for coefficientNumber = 1:6
            inverseSensitivity = conv( ...
                scaledInverse(:, coefficientNumber), hannWindow, "same");
            residual = inverseSensitivity(valid) - ...
                modelCoefficient(valid, coefficientNumber);
            sensitivityMetrics = [sensitivityMetrics; table( ...
                one.Source, one.Name, "thrust_scale", 0.5, ...
                thrustScale, "isa", coefficientNames(coefficientNumber), ...
                mean(residual), sqrt(mean(residual.^2)), ...
                VariableNames=["Source", "Flight", "Sensitivity", ...
                "WindowS", "ThrustScale", "DensitySource", ...
                "Coefficient", "Bias", "RMSE"])]; %#ok<AGROW>
        end
    end

    % 翔仪加速度字段以g为单位；检查9.8与9.80665两种重力常数。
    % PX4字段已经是m/s^2，Gazebo本轮实际重力为9.8，因此不缩放。
    gravitySensitiveSpecificForce = one.SpecificForceFRDMps2;
    if one.Source == "xiangyi"
        gravitySensitiveSpecificForce = ...
            (9.80665 / gravityMps2) * gravitySensitiveSpecificForce;
    end
    gravityForce = massKg * gravitySensitiveSpecificForce - ...
        propellerForceFRD;
    gravityInverse = inverseCoefficientRaw;
    gravityInverse(:, 1) = -sum(gravityForce .* windZ, 2) ./ forceScale;
    gravityInverse(:, 2) = -sum(gravityForce .* windX, 2) ./ forceScale;
    gravityInverse(:, 3) = sum(gravityForce .* windY, 2) ./ forceScale;
    for coefficientNumber = 1:6
        inverseSensitivity = conv( ...
            gravityInverse(:, coefficientNumber), hannWindow, "same");
        residual = inverseSensitivity(valid) - ...
            modelCoefficient(valid, coefficientNumber);
        sensitivityMetrics = [sensitivityMetrics; table( ...
            one.Source, one.Name, "gravity_9_80665", 0.5, 1.0, ...
            "isa", coefficientNames(coefficientNumber), ...
            mean(residual), sqrt(mean(residual.^2)), ...
            VariableNames=["Source", "Flight", "Sensitivity", ...
            "WindowS", "ThrustScale", "DensitySource", ...
            "Coefficient", "Bias", "RMSE"])]; %#ok<AGROW>
    end

    alternateDensity = nan(sampleCount, 1);
    alternateDensityName = "unavailable";
    if one.Source == "px4" && all(isfinite(one.DensityLoggedKgM3))
        alternateDensity = one.DensityLoggedKgM3;
        alternateDensityName = "logged";
    elseif one.Source == "xiangyi"
        alternateDensity = 1.225 * max(0.1, 1 - 2.25577e-5 * ...
            max(-500, min(11000, one.HeightAGLM))).^4.25588;
        alternateDensityName = "navheight_isa";
    end

    if all(isfinite(alternateDensity))
        loggedForceScale = 0.5 * alternateDensity .* ...
            one.TASMps.^2 * areaM2;
        loggedInverse = inverseCoefficientRaw;
        loggedInverse(:, 1) = -sum(aerodynamicForceFRD .* windZ, 2) ./ ...
            loggedForceScale;
        loggedInverse(:, 2) = -sum(aerodynamicForceFRD .* windX, 2) ./ ...
            loggedForceScale;
        loggedInverse(:, 3) = sum(aerodynamicForceFRD .* windY, 2) ./ ...
            loggedForceScale;
        loggedInverse(:, 4) = aerodynamicMomentFRD(:, 1) ./ ...
            (loggedForceScale * spanM);
        loggedInverse(:, 5) = aerodynamicMomentFRD(:, 2) ./ ...
            (loggedForceScale * macM);
        loggedInverse(:, 6) = aerodynamicMomentFRD(:, 3) ./ ...
            (loggedForceScale * spanM);
        for coefficientNumber = 1:6
            inverseSensitivity = conv(loggedInverse(:, coefficientNumber), ...
                hannWindow, "same");
            residual = inverseSensitivity(valid) - ...
                modelCoefficient(valid, coefficientNumber);
            sensitivityMetrics = [sensitivityMetrics; table( ...
                one.Source, one.Name, "density_source", 0.5, 1.0, ...
                alternateDensityName, coefficientNames(coefficientNumber), ...
                mean(residual), sqrt(mean(residual.^2)), ...
                VariableNames=["Source", "Flight", "Sensitivity", ...
                "WindowS", "ThrustScale", "DensitySource", ...
                "Coefficient", "Bias", "RMSE"])]; %#ok<AGROW>
        end
    end

    %% 3.11 保存本架次统一时序
    caseResults(caseIndex).Source = one.Source;
    caseResults(caseIndex).Name = one.Name;
    caseResults(caseIndex).TimeS = one.TimeS;
    caseResults(caseIndex).Valid = valid;
    caseResults(caseIndex).Phase = phase;
    caseResults(caseIndex).InverseCoefficient = inverseCoefficient;
    caseResults(caseIndex).ModelCoefficient = modelCoefficient;
    caseResults(caseIndex).PluginCoefficient = pluginCoefficient;
    caseResults(caseIndex).ThrustN = thrustN;
    caseResults(caseIndex).TorqueNm = torqueNm;
    caseResults(caseIndex).RequiredThrustN = requiredThrust;
    caseResults(caseIndex).RPM = rpm;
    caseResults(caseIndex).DynamicPressurePa = dynamicPressure;
end

disp("统一六分量计算完成。总体指标：")
disp(coefficientMetrics(coefficientMetrics.Phase == "all", :))
