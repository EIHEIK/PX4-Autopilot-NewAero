%% 翔仪仿真与鸿鹄翼 V8 气动模型核对
% 本文件是顺序执行脚本，不包含自定义函数。
% 建议在 MATLAB 编辑器中按 %% 章节逐段运行，也可以直接运行整个脚本。
%
% 坐标约定：
%   翔仪机体系 RFU：X右、Y前、Z上
%   本脚本机体系 FRD：X前、Y右、Z下
%   导航系 NED：X北、Y东、Z下
%
% 翔仪日志已确认的特殊口径：
%   质量 m = 100 kg
%   发动机表油门 = min(1.25 * FW_Thr / 100, 1)
%   发动机反扭矩在 FRD 中取 +Mx
%
% 尚未得到翔仪单独提供的惯量修订值，因此暂时继续使用参数文档中的
% 完整惯量张量。脚本输出属于“在该惯量假设下”的条件性核对结果。

%% 1. 路径、常量和输出目录
if ispc
    flyHome = "\\wsl.localhost\Ubuntu-22.04\home\fly";
else
    flyHome = "/home/fly";
end

repoRoot = fullfile(flyHome, "PX4-Autopilot-NewAero");
csvFile = fullfile(flyHome, "px4_reference_docs", "current", ...
    "翔仪飞控仿真结果.csv");
aeroTableDir = fullfile(repoRoot, "simulation_models", "models", ...
    "honghu_wing_150kg_v8", "aero_tables");
propellerFile = fullfile(repoRoot, "simulation_models", "models", ...
    "honghu_wing_150kg_v8", "propulsion_tables", "propeller.csv");
outputDir = fullfile(repoRoot, "analysis_outputs", ...
    "honghu_v8_xiangyi_matlab");

if ~isfolder(outputDir)
    mkdir(outputDir);
end

massKg = 100.0;
gravity = 9.8;
areaM2 = 2.42;
spanM = 3.96;
macM = 0.62;
sampleTime = 0.05;
thrustDownRad = deg2rad(3.0);
enginePointFRD = [-1.23, 0.0, -0.12];

% 惯量积按照刚体动力学矩阵的符号写入。
inertiaFRD = [ ...
    25.86, -0.017, -3.520; ...
   -0.017, 39.14,  -0.0019; ...
   -3.520, -0.0019, 59.12];

coefficientNames = ["CL", "CD", "CY", "Cl", "Cm", "Cn"];

disp("输入文件：")
disp(csvFile)
disp("输出目录：")
disp(outputDir)

%% 2. 读取翔仪 CSV
% 第一行是577个英文列名，第二行是中文说明，第三行开始是数值数据。
fileID = fopen(csvFile, "r", "n", "GB18030");
assert(fileID >= 0, "无法打开翔仪CSV文件。");
headerLine = fgetl(fileID);
descriptionLine = fgetl(fileID); %#ok<NASGU>
fclose(fileID);

columnNames = string(split(headerLine, ","));
raw = readmatrix(csvFile, "NumHeaderLines", 2, "Encoding", "GB18030");

assert(numel(columnNames) == 577, "英文表头列数不是预期的577列。");
% 原文件每行以逗号结束，第577列始终为空。readmatrix会自动丢弃这个
% 尾部空字段，因此在MATLAB中显式补回，保持与原始577列结构一致。
if size(raw, 2) == 576
    raw(:, 577) = NaN;
end
assert(size(raw, 2) == 577, "翔仪CSV列数不是预期的577列。");

% 只使用以下明确需要的字段。
requiredNames = [ ...
    "IndexPro", "Second", "Millisecond", ...
    "Lat", "Lon", "NavHeight", "NavAltitude", ...
    "V_east", "V_north", "Vz", "TAS", ...
    "Yaw", "Pitch", "Roll", "wX", "wY", "wZ", ...
    "Acc_X", "Acc_Y", "Acc_Z", ...
    "FW_Ail", "FW_Ele", "FW_Thr", "FW_Rud", "Canard"];

columnIndex = zeros(size(requiredNames));
for k = 1:numel(requiredNames)
    found = find(columnNames == requiredNames(k), 1, "first");
    assert(~isempty(found), "CSV中缺少字段：" + requiredNames(k));
    columnIndex(k) = found;
end

data = array2table(raw(:, columnIndex), ...
    "VariableNames", cellstr(requiredNames));
data.SourceRow = (1:height(data))';

disp("CSV数值数据规模：")
disp(size(raw))

%% 3. 选取两次完整有效飞行
% 以下边界来自对完整CSV的架次分割结果，数值为MATLAB的一基索引。
% flight_1存在约35 km经纬度跳变，不进入正式气动统计。
flightNames = ["flight_2", "flight_3"];
flightStart = [14636, 31761];
flightEnd = [27805, 46063];

assert(all(flightStart >= 1) && all(flightEnd <= height(data)), ...
    "有效飞行边界超出了CSV数据范围。");

% IndexPro以模256递增，用于提示丢帧；动力学导数仍采用名义20 Hz时间。
indexCounter = data.IndexPro;
counterStep = mod(diff(indexCounter), 256);
badCounterSteps = find(counterStep ~= 1);
disp("全文件IndexPro非连续步数：")
disp(numel(badCounterSteps))

%% 4. 读取V8静态气动表
staticNames = ["CL", "CD", "CY", "Cl", "Cm", "Cn"];
staticGrid = struct;

for k = 1:numel(staticNames)
    tableFile = fullfile(aeroTableDir, staticNames(k) + ".csv");
    tableCell = readcell(tableFile, "Delimiter", ",", "CommentStyle", "#");
    staticGrid.(staticNames(k)).beta = cell2mat(tableCell(1, 2:end));
    staticGrid.(staticNames(k)).alpha = cell2mat(tableCell(2:end, 1));
    staticGrid.(staticNames(k)).value = cell2mat(tableCell(2:end, 2:end));
end

disp("静态气动表已读取。")

%% 5. 读取V8舵效导数表
controlNames = [ ...
    "aileron_CD", "aileron_CY", "aileron_Cl", "aileron_Cn", ...
    "elevator_CL", "elevator_CD", "elevator_Cm", ...
    "rudder_CD", "rudder_CY", "rudder_Cl", "rudder_Cn", ...
    "canard_CL", "canard_CD", "canard_Cm"];
controlGrid = struct;

for k = 1:numel(controlNames)
    tableFile = fullfile(aeroTableDir, "control_tables", ...
        controlNames(k) + ".csv");
    tableCell = readcell(tableFile, "Delimiter", ",", "CommentStyle", "#");
    controlGrid.(controlNames(k)).column = cell2mat(tableCell(1, 2:end));
    controlGrid.(controlNames(k)).alpha = cell2mat(tableCell(2:end, 1));
    controlGrid.(controlNames(k)).value = cell2mat(tableCell(2:end, 2:end));
end

disp("舵效导数表已读取。")

%% 6. 读取发动机推力和扭矩表
propellerTable = readtable(propellerFile, ...
    "VariableNamingRule", "preserve");
propellerAltitudes = unique(propellerTable.altitude_m);

disp("发动机表海拔层：")
disp(propellerAltitudes')

%% 7. 初始化汇总结果
allTimeseries = table;
allMetrics = table;
allEngineMetrics = table;
frameCheckRows = table;

%% 8. 顺序处理两次完整飞行
for flightNumber = 1:2
    flightName = flightNames(flightNumber);
    rowRange = flightStart(flightNumber):flightEnd(flightNumber);
    flight = data(rowRange, :);
    sampleCount = height(flight);
    timeS = (0:sampleCount-1)' * sampleTime;

    disp("------------------------------------------------------------")
    disp("开始处理：" + flightName)
    disp("样本数：" + sampleCount)

    %% 8.1 RFU角速度和比力转换为FRD
    % 翔仪RFU角速度 [wX,wY,wZ] -> FRD [p,q,r]。
    omegaFRD = deg2rad([ ...
        flight.wY, ...
        flight.wX, ...
       -flight.wZ]);

    % 翔仪加速度字段以g为单位，并且是机体系比力。
    specificForceFRD = gravity * [ ...
        flight.Acc_Y, ...
        flight.Acc_X, ...
       -flight.Acc_Z];

    %% 8.2 NED速度转换到FRD机体系
    rollRad = deg2rad(flight.Roll);
    pitchRad = deg2rad(flight.Pitch);
    yawRad = unwrap(deg2rad(flight.Yaw));

    cRoll = cos(rollRad);
    sRoll = sin(rollRad);
    cPitch = cos(pitchRad);
    sPitch = sin(pitchRad);
    cYaw = cos(yawRad);
    sYaw = sin(yawRad);

    velocityNED = [ ...
        flight.V_north, ...
        flight.V_east, ...
       -flight.Vz];

    % R_b_to_n采用标准ZYX偏航-俯仰-滚转序列。
    uBody = ...
        cPitch .* cYaw .* velocityNED(:, 1) + ...
        cPitch .* sYaw .* velocityNED(:, 2) - ...
        sPitch .* velocityNED(:, 3);

    vBody = ...
        (sRoll .* sPitch .* cYaw - cRoll .* sYaw) .* velocityNED(:, 1) + ...
        (sRoll .* sPitch .* sYaw + cRoll .* cYaw) .* velocityNED(:, 2) + ...
        sRoll .* cPitch .* velocityNED(:, 3);

    wBody = ...
        (cRoll .* sPitch .* cYaw + sRoll .* sYaw) .* velocityNED(:, 1) + ...
        (cRoll .* sPitch .* sYaw - sRoll .* cYaw) .* velocityNED(:, 2) + ...
        cRoll .* cPitch .* velocityNED(:, 3);

    velocityBodyFRD = [uBody, vBody, wBody];
    bodySpeed = vecnorm(velocityBodyFRD, 2, 2);
    alphaRad = atan2(wBody, uBody);
    betaRad = atan2(vBody, hypot(uBody, wBody));
    alphaDeg = rad2deg(alphaRad);
    betaDeg = rad2deg(betaRad);

    %% 8.3 坐标系自检
    % 日志欧拉角存在0.01 deg量化，先用1 s对称Hann窗平滑再求导。
    checkWindowSamples = round(1.0 / sampleTime);
    if mod(checkWindowSamples, 2) == 0
        checkWindowSamples = checkWindowSamples + 1;
    end
    checkWindowIndex = (0:checkWindowSamples-1)';
    checkWindow = 0.5 - 0.5 * cos( ...
        2 * pi * checkWindowIndex / (checkWindowSamples - 1));
    checkWindow = checkWindow / sum(checkWindow);

    rollSmooth = conv(rollRad, checkWindow, "same");
    pitchSmooth = conv(pitchRad, checkWindow, "same");
    yawSmooth = conv(yawRad, checkWindow, "same");
    rollDot = gradient(rollSmooth, sampleTime);
    pitchDot = gradient(pitchSmooth, sampleTime);
    yawDot = gradient(yawSmooth, sampleTime);

    omegaFromEuler = [ ...
        rollDot - yawDot .* sin(pitchRad), ...
        pitchDot .* cos(rollRad) + yawDot .* sin(rollRad) .* cos(pitchRad), ...
       -pitchDot .* sin(rollRad) + yawDot .* cos(rollRad) .* cos(pitchRad)];

    checkMask = flight.TAS >= 20 & flight.NavHeight >= 5 & ...
        all(isfinite(omegaFromEuler), 2);
    checkMask(1:checkWindowSamples) = false;
    checkMask(end-checkWindowSamples+1:end) = false;

    rateCorrelation = zeros(1, 3);
    for axisNumber = 1:3
        correlationMatrix = corrcoef( ...
            omegaFromEuler(checkMask, axisNumber), ...
            omegaFRD(checkMask, axisNumber));
        rateCorrelation(axisNumber) = correlationMatrix(1, 2);
    end

    speedRmse = sqrt(mean( ...
        (bodySpeed(checkMask) - flight.TAS(checkMask)).^2));

    % 用导航速度导数独立重建机体系比力。
    velocityNedSmooth = zeros(size(velocityNED));
    accelerationNED = zeros(size(velocityNED));
    for axisNumber = 1:3
        velocityNedSmooth(:, axisNumber) = conv( ...
            velocityNED(:, axisNumber), checkWindow, "same");
        accelerationNED(:, axisNumber) = gradient( ...
            velocityNedSmooth(:, axisNumber), sampleTime);
    end
    accelerationMinusGravity = accelerationNED - ...
        [zeros(sampleCount, 2), gravity * ones(sampleCount, 1)];

    forceCheckX = ...
        cPitch .* cYaw .* accelerationMinusGravity(:, 1) + ...
        cPitch .* sYaw .* accelerationMinusGravity(:, 2) - ...
        sPitch .* accelerationMinusGravity(:, 3);
    forceCheckY = ...
        (sRoll .* sPitch .* cYaw - cRoll .* sYaw) .* ...
            accelerationMinusGravity(:, 1) + ...
        (sRoll .* sPitch .* sYaw + cRoll .* cYaw) .* ...
            accelerationMinusGravity(:, 2) + ...
        sRoll .* cPitch .* accelerationMinusGravity(:, 3);
    forceCheckZ = ...
        (cRoll .* sPitch .* cYaw + sRoll .* sYaw) .* ...
            accelerationMinusGravity(:, 1) + ...
        (cRoll .* sPitch .* sYaw - sRoll .* cYaw) .* ...
            accelerationMinusGravity(:, 2) + ...
        cRoll .* cPitch .* accelerationMinusGravity(:, 3);
    kinematicSpecificForce = [forceCheckX, forceCheckY, forceCheckZ];

    forceCorrelation = zeros(1, 3);
    for axisNumber = 1:3
        correlationMatrix = corrcoef( ...
            kinematicSpecificForce(checkMask, axisNumber), ...
            specificForceFRD(checkMask, axisNumber));
        forceCorrelation(axisNumber) = correlationMatrix(1, 2);
    end

    disp("角速度相关系数[p,q,r]：")
    disp(rateCorrelation)
    disp("比力相关系数[fx,fy,fz]：")
    disp(forceCorrelation)
    disp("TAS与机体系速度模RMSE / (m/s)：")
    disp(speedRmse)

    frameCheckRows = [frameCheckRows; table( ...
        flightName, rateCorrelation(1), rateCorrelation(2), ...
        rateCorrelation(3), forceCorrelation(1), forceCorrelation(2), ...
        forceCorrelation(3), speedRmse, ...
        VariableNames=["Flight", "CorrP", "CorrQ", "CorrR", ...
        "CorrFx", "CorrFy", "CorrFz", "TasBodySpeedRMSE"])]; %#ok<AGROW>

    assert(all(rateCorrelation >= 0.85), ...
        flightName + "的欧拉角/角速度坐标自检未通过。");
    assert(all(forceCorrelation >= 0.90), ...
        flightName + "的导航运动学/比力坐标自检未通过。");
    assert(speedRmse <= 0.1, ...
        flightName + "的TAS/机体系速度自检未通过。");

    %% 8.4 计算alpha_dot和beta_dot
    % 与V8插件一致，导数先限幅，再使用0.05 s一阶滤波。
    alphaDot = zeros(sampleCount, 1);
    betaDot = zeros(sampleCount, 1);
    filteredAlphaDot = 0;
    filteredBetaDot = 0;

    for sample = 2:sampleCount
        rawAlphaDot = (alphaRad(sample) - alphaRad(sample-1)) / sampleTime;
        rawBetaDot = (betaRad(sample) - betaRad(sample-1)) / sampleTime;
        rawAlphaDot = max(-10, min(10, rawAlphaDot));
        rawBetaDot = max(-10, min(10, rawBetaDot));
        filterGain = sampleTime / (0.05 + sampleTime);
        filteredAlphaDot = filteredAlphaDot + ...
            filterGain * (rawAlphaDot - filteredAlphaDot);
        filteredBetaDot = filteredBetaDot + ...
            filterGain * (rawBetaDot - filteredBetaDot);
        alphaDot(sample) = filteredAlphaDot;
        betaDot(sample) = filteredBetaDot;
    end

    %% 8.5 计算V8静态气动系数
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

        % V8对表格最低迎角以下使用前两行斜率线性延伸。
        lowAlphaMask = alphaDeg < rowAxis(1);
        if any(lowAlphaMask)
            valueAtFirstRow = interp2(columnAxis, rowAxis, valueGrid, ...
                queryBeta(lowAlphaMask), ...
                rowAxis(1) * ones(nnz(lowAlphaMask), 1), "linear");
            valueAtSecondRow = interp2(columnAxis, rowAxis, valueGrid, ...
                queryBeta(lowAlphaMask), ...
                rowAxis(2) * ones(nnz(lowAlphaMask), 1), "linear");
            rowSlope = (valueAtSecondRow - valueAtFirstRow) / ...
                (rowAxis(2) - rowAxis(1));
            coefficientValue(lowAlphaMask) = valueAtFirstRow + ...
                rowSlope .* (alphaDeg(lowAlphaMask) - rowAxis(1));
        end

        if any(coefficientName == ["CY", "Cl", "Cn"])
            coefficientValue = coefficientValue .* lateralSign;
        end
        staticCoefficient(:, coefficientNumber) = coefficientValue;
    end

    %% 8.6 计算副翼舵效
    deltaA = flight.FW_Ail;
    deltaALookup = max(-10, min(10, deltaA));
    fadeInput = max(0, min(1, (abs(alphaDeg) - 12) / 8));
    aileronFade = 1 - fadeInput.^2 .* (3 - 2 * fadeInput);
    aileronContribution = zeros(sampleCount, 6);

    aileronTargets = ["CD", "CY", "Cl", "Cn"];
    aileronColumns = [2, 3, 4, 6];
    for k = 1:numel(aileronTargets)
        gridName = "aileron_" + aileronTargets(k);
        rowAxis = controlGrid.(gridName).alpha;
        columnAxis = controlGrid.(gridName).column;
        valueGrid = controlGrid.(gridName).value;
        queryAlpha = max(rowAxis(1), min(rowAxis(end), alphaDeg));
        derivative = interp2(columnAxis, rowAxis, valueGrid, ...
            deltaALookup, queryAlpha, "linear");
        aileronContribution(:, aileronColumns(k)) = ...
            derivative .* deltaA .* aileronFade;
    end

    %% 8.7 计算升降舵舵效
    deltaE = flight.FW_Ele;
    deltaELookup = max(-10, min(20, deltaE));
    elevatorContribution = zeros(sampleCount, 6);

    elevatorTargets = ["CL", "CD", "Cm"];
    elevatorColumns = [1, 2, 5];
    for k = 1:numel(elevatorTargets)
        gridName = "elevator_" + elevatorTargets(k);
        rowAxis = controlGrid.(gridName).alpha;
        columnAxis = controlGrid.(gridName).column;
        valueGrid = controlGrid.(gridName).value;
        queryAlpha = max(rowAxis(1), min(rowAxis(end), alphaDeg));
        derivative = interp2(columnAxis, rowAxis, valueGrid, ...
            deltaELookup, queryAlpha, "linear");
        elevatorContribution(:, elevatorColumns(k)) = derivative .* deltaE;
    end

    %% 8.8 计算方向舵舵效
    deltaR = flight.FW_Rud;
    reflectedBeta = betaDeg;
    reflectedBeta(deltaR < 0) = -reflectedBeta(deltaR < 0);
    rudderContribution = zeros(sampleCount, 6);

    rudderTargets = ["CD", "CY", "Cl", "Cn"];
    rudderColumns = [2, 3, 4, 6];
    for k = 1:numel(rudderTargets)
        gridName = "rudder_" + rudderTargets(k);
        rowAxis = controlGrid.(gridName).alpha;
        columnAxis = controlGrid.(gridName).column;
        valueGrid = controlGrid.(gridName).value;
        queryAlpha = max(rowAxis(1), min(rowAxis(end), alphaDeg));
        queryBeta = max(columnAxis(1), min(columnAxis(end), reflectedBeta));
        derivative = interp2(columnAxis, rowAxis, valueGrid, ...
            queryBeta, queryAlpha, "linear");
        if rudderTargets(k) == "CD"
            rudderCdSign = ones(sampleCount, 1);
            rudderCdSign(deltaR < 0) = -1;
            derivative = derivative .* rudderCdSign;
        end
        rudderContribution(:, rudderColumns(k)) = derivative .* deltaR;
    end

    %% 8.9 计算鸭翼舵效
    deltaC = flight.Canard;
    deltaCEffective = max(-4, min(15, deltaC));
    deltaCEffective(deltaC < -4) = -4;
    deltaCLookup = max(-4, min(8, deltaCEffective));
    fadeInput = max(0, min(1, (abs(alphaDeg + deltaCEffective) - 12) / 4));
    canardFade = 1 - fadeInput.^2 .* (3 - 2 * fadeInput);
    canardContribution = zeros(sampleCount, 6);

    canardTargets = ["CL", "CD", "Cm"];
    canardColumns = [1, 2, 5];
    for k = 1:numel(canardTargets)
        gridName = "canard_" + canardTargets(k);
        rowAxis = controlGrid.(gridName).alpha;
        columnAxis = controlGrid.(gridName).column;
        valueGrid = controlGrid.(gridName).value;
        queryAlpha = max(rowAxis(1), min(rowAxis(end), alphaDeg));
        derivative = interp2(columnAxis, rowAxis, valueGrid, ...
            deltaCLookup, queryAlpha, "linear");
        canardContribution(:, canardColumns(k)) = ...
            derivative .* deltaCEffective .* canardFade;
    end

    %% 8.10 加入V8动导数
    v8Coefficient = staticCoefficient + aileronContribution + ...
        elevatorContribution + rudderContribution + canardContribution;

    speedForDerivative = max(flight.TAS, 5);
    rateBlendInput = max(0, min(1, (flight.TAS - 3) / 2));
    rateBlend = rateBlendInput.^2 .* (3 - 2 * rateBlendInput);
    inverseTwoV = 0.5 ./ speedForDerivative;

    pRate = omegaFRD(:, 1);
    qRate = omegaFRD(:, 2);
    rRate = omegaFRD(:, 3);

    v8Coefficient(:, 1) = v8Coefficient(:, 1) + rateBlend .* ...
        5.62 .* qRate .* macM .* inverseTwoV;
    v8Coefficient(:, 3) = v8Coefficient(:, 3) + rateBlend .* ( ...
       -0.15 .* pRate .* spanM .* inverseTwoV + ...
        0.34 .* rRate .* spanM .* inverseTwoV);
    v8Coefficient(:, 4) = v8Coefficient(:, 4) + rateBlend .* ( ...
       -0.33 .* pRate .* spanM .* inverseTwoV + ...
        0.10 .* rRate .* spanM .* inverseTwoV);
    v8Coefficient(:, 5) = v8Coefficient(:, 5) + rateBlend .* ( ...
       -7.00 .* qRate .* macM .* inverseTwoV - ...
        0.33 .* alphaDot .* macM .* inverseTwoV);
    v8Coefficient(:, 6) = v8Coefficient(:, 6) + rateBlend .* ( ...
       -0.05 .* pRate .* spanM .* inverseTwoV - ...
        0.08 .* rRate .* spanM .* inverseTwoV + ...
        0.14 .* betaDot .* spanM .* inverseTwoV);

    %% 8.11 发动机输入映射
    loggedThrottle = max(0, min(1, flight.FW_Thr / 100));
    mappedThrottle = max(0, min(1, 1.25 * loggedThrottle));

    % 两列分别为修正后的1.25倍映射和错误的直接映射，用于对照。
    throttleCases = [mappedThrottle, loggedThrottle];
    thrustCases = zeros(sampleCount, 2);
    torqueCases = zeros(sampleCount, 2);
    rpmCases = zeros(sampleCount, 2);

    %% 8.12 按海拔、油门和空速顺序插值发动机表
    % 这里刻意直接展开查表过程，便于逐行检查，不隐藏到函数中。
    for throttleCase = 1:2
        for sample = 1:sampleCount
            altitudeQuery = max(propellerAltitudes(1), ...
                min(propellerAltitudes(end), flight.NavHeight(sample)));
            speedQuery = max(0, min(50, flight.TAS(sample)));
            throttlePercentQuery = 100 * throttleCases(sample, throttleCase);

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
                throttleAtLevel = max(0, ...
                    min(throttleLevels(end), throttlePercentQuery));

                throttleHigh = find(throttleLevels > throttleAtLevel, ...
                    1, "first");
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
                        resultAtThrottle(throttleSlot, 3) = ...
                            tableRows.rpm(1);
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
            finalEngineResult = resultAtAltitude(1, :) + altitudeFraction * ...
                (resultAtAltitude(2, :) - resultAtAltitude(1, :));

            thrustCases(sample, throttleCase) = finalEngineResult(1);
            torqueCases(sample, throttleCase) = finalEngineResult(2);
            rpmCases(sample, throttleCase) = finalEngineResult(3);
        end
    end

    thrustN = thrustCases(:, 1);
    engineTorqueNm = torqueCases(:, 1);

    %% 8.13 发动机力和力矩
    thrustDirectionFRD = [cos(thrustDownRad), 0, sin(thrustDownRad)];
    propellerForceFRD = thrustN .* thrustDirectionFRD;

    % 翔仪数据支持机体反扭矩为 +Mx_FRD。
    reactionMomentFRD = [engineTorqueNm, ...
        zeros(sampleCount, 1), zeros(sampleCount, 1)];
    propellerMomentFRD = cross( ...
        repmat(enginePointFRD, sampleCount, 1), ...
        propellerForceFRD, 2) + reactionMomentFRD;

    %% 8.14 用刚体动力学反算气动力和气动力矩
    % 使用0.5 s对称Hann窗平滑角速度，再求角加速度。
    filterSamples = round(0.5 / sampleTime);
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
            omegaFiltered(:, axisNumber), sampleTime);
    end

    aerodynamicForceFRD = ...
        massKg * specificForceFRD - propellerForceFRD;

    inertiaOmega = omegaFiltered * inertiaFRD';
    totalMomentFRD = omegaDot * inertiaFRD' + ...
        cross(omegaFiltered, inertiaOmega, 2);
    aerodynamicMomentFRD = totalMomentFRD - propellerMomentFRD;

    %% 8.15 把反算气动力转换成六分量无量纲系数
    density = 1.225 * max(0.1, ...
        1 - 2.25577e-5 * max(-500, min(11000, ...
        flight.NavAltitude))).^4.25588;
    dynamicPressure = 0.5 * density .* flight.TAS.^2;
    forceScale = dynamicPressure * areaM2;

    cosAlpha = cos(alphaRad);
    sinAlpha = sin(alphaRad);
    cosBeta = cos(betaRad);
    sinBeta = sin(betaRad);

    windX = [ ...
        cosAlpha .* cosBeta, ...
        sinBeta, ...
        sinAlpha .* cosBeta];
    windY = [ ...
       -cosAlpha .* sinBeta, ...
        cosBeta, ...
       -sinAlpha .* sinBeta];
    windZ = [-sinAlpha, zeros(sampleCount, 1), cosAlpha];

    inverseCoefficient = zeros(sampleCount, 6);
    inverseCoefficient(:, 1) = -sum( ...
        aerodynamicForceFRD .* windZ, 2) ./ forceScale;
    inverseCoefficient(:, 2) = -sum( ...
        aerodynamicForceFRD .* windX, 2) ./ forceScale;
    inverseCoefficient(:, 3) = sum( ...
        aerodynamicForceFRD .* windY, 2) ./ forceScale;
    inverseCoefficient(:, 4) = aerodynamicMomentFRD(:, 1) ./ ...
        (forceScale * spanM);
    inverseCoefficient(:, 5) = aerodynamicMomentFRD(:, 2) ./ ...
        (forceScale * macM);
    inverseCoefficient(:, 6) = aerodynamicMomentFRD(:, 3) ./ ...
        (forceScale * spanM);

    %% 8.16 有效样本和0.5 s系数平滑
    edgeMask = false(sampleCount, 1);
    halfWindow = floor(filterSamples / 2);
    edgeMask(halfWindow+1:end-halfWindow) = true;

    valid = ...
        flight.TAS >= 20 & flight.TAS <= 50 & ...
        flight.NavHeight >= 5 & dynamicPressure >= 200 & ...
        alphaDeg >= -2 & alphaDeg <= 20 & ...
        abs(betaDeg) <= 16 & edgeMask & ...
        all(isfinite(inverseCoefficient), 2) & ...
        all(isfinite(v8Coefficient), 2);

    inverseFiltered = zeros(size(inverseCoefficient));
    v8Filtered = zeros(size(v8Coefficient));
    for coefficientNumber = 1:6
        inverseFiltered(:, coefficientNumber) = conv( ...
            inverseCoefficient(:, coefficientNumber), ...
            hannWindow, "same");
        v8Filtered(:, coefficientNumber) = conv( ...
            v8Coefficient(:, coefficientNumber), ...
            hannWindow, "same");
    end

    %% 8.17 计算六分量对比指标
    for coefficientNumber = 1:6
        inverseValue = inverseFiltered(valid, coefficientNumber);
        v8Value = v8Filtered(valid, coefficientNumber);
        residual = inverseValue - v8Value;

        biasValue = mean(residual);
        rmseValue = sqrt(mean(residual.^2));
        maeValue = mean(abs(residual));
        correlationMatrix = corrcoef(inverseValue, v8Value);
        correlationValue = correlationMatrix(1, 2);
        linearFit = polyfit(v8Value, inverseValue, 1);

        allMetrics = [allMetrics; table( ...
            flightName, coefficientNames(coefficientNumber), ...
            nnz(valid), mean(inverseValue), mean(v8Value), ...
            biasValue, rmseValue, maeValue, correlationValue, ...
            linearFit(1), ...
            VariableNames=["Flight", "Coefficient", "Samples", ...
            "InverseMean", "V8Mean", "Bias", "RMSE", "MAE", ...
            "Correlation", "Slope"])]; %#ok<AGROW>
    end

    %% 8.18 发动机闭合检查
    % 先由气动正向模型计算气动力，再由总比力反推出所需发动机力。
    aerodynamicForceFromV8 = ...
       -v8Coefficient(:, 2) .* forceScale .* windX + ...
        v8Coefficient(:, 3) .* forceScale .* windY - ...
        v8Coefficient(:, 1) .* forceScale .* windZ;
    requiredPropellerForce = ...
        massKg * specificForceFRD - aerodynamicForceFromV8;
    requiredThrust = requiredPropellerForce * thrustDirectionFRD';

    aerodynamicMomentFromV8 = [ ...
        v8Coefficient(:, 4) .* forceScale * spanM, ...
        v8Coefficient(:, 5) .* forceScale * macM, ...
        v8Coefficient(:, 6) .* forceScale * spanM];
    requiredPropellerMoment = totalMomentFRD - aerodynamicMomentFromV8;
    thrustPointMoment = cross( ...
        repmat(enginePointFRD, sampleCount, 1), ...
        propellerForceFRD, 2);
    requiredReactionMomentX = requiredPropellerMoment(:, 1) - ...
        thrustPointMoment(:, 1);

    cruiseMask = valid & flight.TAS >= 43 & flight.TAS <= 46 & ...
        abs(flight.Roll) <= 3 & abs(flight.Vz) <= 0.2 & ...
        flight.FW_Thr >= 63 & flight.FW_Thr <= 67;

    mappedResidual = thrustCases(cruiseMask, 1) - ...
        requiredThrust(cruiseMask);
    directResidual = thrustCases(cruiseMask, 2) - ...
        requiredThrust(cruiseMask);
    torqueResidual = engineTorqueNm(cruiseMask) - ...
        requiredReactionMomentX(cruiseMask);

    allEngineMetrics = [allEngineMetrics; table( ...
        flightName, nnz(cruiseMask), ...
        mean(requiredThrust(cruiseMask)), ...
        mean(thrustCases(cruiseMask, 1)), ...
        sqrt(mean(mappedResidual.^2)), ...
        mean(thrustCases(cruiseMask, 2)), ...
        sqrt(mean(directResidual.^2)), ...
        mean(requiredReactionMomentX(cruiseMask)), ...
        mean(engineTorqueNm(cruiseMask)), ...
        mean(torqueResidual), sqrt(mean(torqueResidual.^2)), ...
        VariableNames=["Flight", "CruiseSamples", ...
        "RequiredThrustN", "MappedTableThrustN", ...
        "MappedThrustRMSE", "DirectTableThrustN", ...
        "DirectThrustRMSE", "RequiredReactionMomentXNm", ...
        "MappedTableTorqueNm", "TorqueBiasNm", ...
        "TorqueRMSE"])]; %#ok<AGROW>

    %% 8.19 保存本架次逐样本结果
    flightColumn = repmat(flightName, sampleCount, 1);
    oneFlightResult = table;
    oneFlightResult.Flight = flightColumn;
    oneFlightResult.SourceRow = flight.SourceRow;
    oneFlightResult.TimeS = timeS;
    oneFlightResult.PlotTimeS = timeS + (flightNumber - 1) * 800;
    oneFlightResult.Valid = valid;
    oneFlightResult.LatitudeDeg = flight.Lat;
    oneFlightResult.LongitudeDeg = flight.Lon;
    oneFlightResult.HeightAGLM = flight.NavHeight;
    oneFlightResult.AltitudeMSLM = flight.NavAltitude;
    oneFlightResult.TASMps = flight.TAS;
    oneFlightResult.YawDeg = flight.Yaw;
    oneFlightResult.PitchDeg = flight.Pitch;
    oneFlightResult.RollDeg = flight.Roll;
    oneFlightResult.VerticalSpeedMps = flight.Vz;
    oneFlightResult.AlphaDeg = alphaDeg;
    oneFlightResult.BetaDeg = betaDeg;
    oneFlightResult.PRadS = omegaFRD(:, 1);
    oneFlightResult.QRadS = omegaFRD(:, 2);
    oneFlightResult.RRadS = omegaFRD(:, 3);
    oneFlightResult.DeltaADeg = deltaA;
    oneFlightResult.DeltaEDeg = deltaE;
    oneFlightResult.DeltaRDeg = deltaR;
    oneFlightResult.DeltaCDeg = deltaC;
    oneFlightResult.LoggedThrottle = loggedThrottle;
    oneFlightResult.MappedTableThrottle = mappedThrottle;
    oneFlightResult.ThrustN = thrustN;
    oneFlightResult.EngineTorqueNm = engineTorqueNm;
    oneFlightResult.RequiredThrustN = requiredThrust;

    for coefficientNumber = 1:6
        oneFlightResult.(coefficientNames(coefficientNumber) + ...
            "_Inverse") = inverseFiltered(:, coefficientNumber);
        oneFlightResult.(coefficientNames(coefficientNumber) + ...
            "_V8") = v8Filtered(:, coefficientNumber);
    end

    allTimeseries = [allTimeseries; oneFlightResult]; %#ok<AGROW>
end

%% 9. 输出表格文件
writetable(frameCheckRows, fullfile(outputDir, ...
    "matlab_frame_checks.csv"));
writetable(allMetrics, fullfile(outputDir, ...
    "matlab_coefficient_metrics.csv"));
writetable(allEngineMetrics, fullfile(outputDir, ...
    "matlab_engine_metrics.csv"));
writetable(allTimeseries, fullfile(outputDir, ...
    "matlab_coefficient_timeseries.csv"));

disp("坐标系自检：")
disp(frameCheckRows)
disp("六分量对比指标：")
disp(allMetrics)
disp("发动机闭合指标：")
disp(allEngineMetrics)

%% 10. 绘制六分量时序对比图
plotMask = allTimeseries.Valid;
plotData = allTimeseries(plotMask, :);

figCoefficient = figure("Visible", "off", ...
    "Color", "white", "Position", [100, 100, 1500, 900]);
layoutCoefficient = tiledlayout(figCoefficient, 3, 2, ...
    "TileSpacing", "compact", "Padding", "compact");

for coefficientNumber = 1:6
    axisHandle = nexttile(layoutCoefficient);
    inverseName = coefficientNames(coefficientNumber) + "_Inverse";
    v8Name = coefficientNames(coefficientNumber) + "_V8";
    plot(axisHandle, plotData.PlotTimeS, plotData.(inverseName), ...
        "Color", [0.00, 0.45, 0.74], "LineWidth", 1.0);
    hold(axisHandle, "on");
    plot(axisHandle, plotData.PlotTimeS, plotData.(v8Name), ...
        "Color", [0.85, 0.33, 0.10], "LineWidth", 1.0);
    hold(axisHandle, "off");
    grid(axisHandle, "on");
    xlabel(axisHandle, "两架次拼接显示时间 / s");
    ylabel(axisHandle, coefficientNames(coefficientNumber));
    title(axisHandle, coefficientNames(coefficientNumber) + ...
        "：动力学反算与V8正向计算");
    legend(axisHandle, ["动力学反算", "V8正向模型"], ...
        "Location", "best");
end

title(layoutCoefficient, ...
    "翔仪两次完整飞行：六分量气动系数核对");
exportgraphics(figCoefficient, fullfile(outputDir, ...
    "matlab_coefficient_comparison.png"), "Resolution", 200);
close(figCoefficient);

%% 11. 绘制发动机映射检查图
cruisePlotMask = plotData.LoggedThrottle >= 0.63 & ...
    plotData.LoggedThrottle <= 0.67 & ...
    plotData.TASMps >= 43 & plotData.TASMps <= 46 & ...
    abs(plotData.RollDeg) <= 3 & ...
    abs(plotData.VerticalSpeedMps) <= 0.2;

figEngine = figure("Visible", "off", ...
    "Color", "white", "Position", [100, 100, 1200, 600]);
axisEngine = axes(figEngine);
scatter(axisEngine, ...
    plotData.RequiredThrustN(cruisePlotMask), ...
    plotData.ThrustN(cruisePlotMask), ...
    10, plotData.TASMps(cruisePlotMask), "filled");
hold(axisEngine, "on");
engineLimits = [ ...
    min(plotData.RequiredThrustN(cruisePlotMask)), ...
    max(plotData.RequiredThrustN(cruisePlotMask))];
plot(axisEngine, engineLimits, engineLimits, "k--", "LineWidth", 1.2);
hold(axisEngine, "off");
grid(axisEngine, "on");
xlabel(axisEngine, "动力学所需推力 / N");
ylabel(axisEngine, "1.25倍油门映射后的表格推力 / N");
title(axisEngine, "翔仪巡航发动机推力闭合");
colorbar(axisEngine);
exportgraphics(figEngine, fullfile(outputDir, ...
    "matlab_engine_thrust_closure.png"), "Resolution", 200);
close(figEngine);

%% 12. 保存MAT文件和JSON摘要
save(fullfile(outputDir, "matlab_xiangyi_v8_aero_results.mat"), ...
    "frameCheckRows", "allMetrics", "allEngineMetrics", ...
    "allTimeseries", "massKg", "inertiaFRD");

summary = struct;
summary.generatedBy = "MATLAB R2025a sequential script";
summary.massKg = massKg;
summary.inertiaAssumption = ...
    "PDF/V8 inertia retained because Xiangyi revised inertia was not provided";
summary.engineThrottleMapping = ...
    "u_table = min(1.25 * FW_Thr / 100, 1)";
summary.engineReactionTorque = "+Mx_FRD";
summary.frameChecks = table2struct(frameCheckRows);
summary.coefficientMetrics = table2struct(allMetrics);
summary.engineMetrics = table2struct(allEngineMetrics);

jsonText = jsonencode(summary, "PrettyPrint", true);
writelines(jsonText, fullfile(outputDir, ...
    "matlab_comparison_summary.json"));

disp("分析完成。结果已写入：")
disp(outputDir)
