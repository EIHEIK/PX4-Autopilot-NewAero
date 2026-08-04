%% 4. 导出六分量表格、图片、MAT文件和JSON摘要

writetable(frameChecks, fullfile(aeroOutputDir, "frame_checks.csv"));
writetable(coefficientMetrics, fullfile(aeroOutputDir, ...
    "coefficient_metrics.csv"));
writetable(pluginMetrics, fullfile(aeroOutputDir, ...
    "px4_plugin_crosscheck_metrics.csv"));
writetable(engineMetrics, fullfile(aeroOutputDir, "engine_metrics.csv"));
writetable(sensitivityMetrics, fullfile(aeroOutputDir, ...
    "sensitivity_metrics.csv"));

allTimeseries = table;
caseColors = [ ...
    0.0000, 0.4470, 0.7410; ...
    0.0000, 0.6200, 0.4500; ...
    0.8500, 0.3250, 0.0980; ...
    0.6350, 0.0780, 0.1840];

for caseIndex = 1:numel(flightCases)
    one = flightCases(caseIndex);
    result = caseResults(caseIndex);
    sampleCount = numel(one.TimeS);
    oneTable = table;
    oneTable.Source = repmat(one.Source, sampleCount, 1);
    oneTable.Flight = repmat(one.Name, sampleCount, 1);
    oneTable.TimeS = one.TimeS;
    oneTable.Valid = result.Valid;
    oneTable.Phase = result.Phase;
    oneTable.LatitudeDeg = one.LatitudeDeg;
    oneTable.LongitudeDeg = one.LongitudeDeg;
    oneTable.HeightAGLM = one.HeightAGLM;
    oneTable.AltitudeMSLM = one.AltitudeMSLM;
    oneTable.TASMps = one.TASMps;
    oneTable.YawDeg = one.YawDeg;
    oneTable.PitchDeg = one.PitchDeg;
    oneTable.RollDeg = one.RollDeg;
    oneTable.VerticalSpeedMps = one.VerticalSpeedMps;
    oneTable.AlphaDeg = one.AlphaDeg;
    oneTable.BetaDeg = one.BetaDeg;
    oneTable.PRadS = one.OmegaFRDRadS(:, 1);
    oneTable.QRadS = one.OmegaFRDRadS(:, 2);
    oneTable.RRadS = one.OmegaFRDRadS(:, 3);
    oneTable.DeltaADeg = one.DeltaDocDeg(:, 1);
    oneTable.DeltaEDeg = one.DeltaDocDeg(:, 2);
    oneTable.DeltaRDeg = one.DeltaDocDeg(:, 3);
    oneTable.DeltaCDeg = one.DeltaDocDeg(:, 4);
    oneTable.ThrottleTarget = one.ThrottleTarget;
    oneTable.ThrottleState = one.ThrottleState;
    oneTable.ThrustN = result.ThrustN;
    oneTable.RequiredThrustN = result.RequiredThrustN;
    oneTable.TorqueNm = result.TorqueNm;
    oneTable.MissionSeq = one.MissionSeq;
    oneTable.Landed = one.Landed;

    for coefficientNumber = 1:6
        coefficientName = coefficientNames(coefficientNumber);
        oneTable.(coefficientName + "_Inverse") = ...
            result.InverseCoefficient(:, coefficientNumber);
        oneTable.(coefficientName + "_Model") = ...
            result.ModelCoefficient(:, coefficientNumber);
        oneTable.(coefficientName + "_Plugin") = ...
            result.PluginCoefficient(:, coefficientNumber);
        oneTable.(coefficientName + "_Residual") = ...
            result.InverseCoefficient(:, coefficientNumber) - ...
            result.ModelCoefficient(:, coefficientNumber);
    end
    allTimeseries = [allTimeseries; oneTable]; %#ok<AGROW>

    figureHandle = figure("Visible", "off", ...
        "Position", [100, 100, 1500, 900]);
    layout = tiledlayout(figureHandle, 3, 2, ...
        "TileSpacing", "compact", "Padding", "compact");
    for coefficientNumber = 1:6
        axisHandle = nexttile(layout);
        % 动力学反算在起飞前、接地后以及动压接近零时没有物理意义，
        % 除以很小的qS会把微小的加速度噪声放大到1e10量级。原始结果
        % 仍完整写入CSV；图片只显示step_03定义的有效空中样本，并以NaN
        % 断开无效区间，避免把跨越地面阶段的点错误连成直线。
        inverseForPlot = result.InverseCoefficient(:, coefficientNumber);
        modelForPlot = result.ModelCoefficient(:, coefficientNumber);
        pluginForPlot = result.PluginCoefficient(:, coefficientNumber);
        plotMask = result.Valid & isfinite(one.TimeS) & ...
            isfinite(inverseForPlot) & isfinite(modelForPlot);
        if one.Source == "px4"
            plotMask = plotMask & isfinite(pluginForPlot);
        end
        inverseForPlot(~plotMask) = NaN;
        modelForPlot(~plotMask) = NaN;
        pluginForPlot(~plotMask) = NaN;

        plot(axisHandle, one.TimeS, ...
            inverseForPlot, ...
            "Color", [0.10, 0.10, 0.10], "LineWidth", 0.9, ...
            "DisplayName", "动力学反算");
        hold(axisHandle, "on");
        plot(axisHandle, one.TimeS, ...
            modelForPlot, ...
            "Color", caseColors(caseIndex, :), "LineWidth", 1.0, ...
            "DisplayName", "MATLAB V8正向");
        if one.Source == "px4"
            plot(axisHandle, one.TimeS, ...
                pluginForPlot, "--", ...
                "Color", [0.45, 0.45, 0.45], "LineWidth", 0.8, ...
                "DisplayName", "Gazebo插件");
        end

        % 有效样本中仍可能存在个别导数尖峰。纵轴按0.5%～99.5%的
        % 稳健范围设置，只影响显示，不删除CSV数据和统计用样本。
        axisValues = [inverseForPlot(plotMask); modelForPlot(plotMask)];
        if one.Source == "px4"
            axisValues = [axisValues; pluginForPlot(plotMask)]; %#ok<AGROW>
        end
        axisValues = sort(axisValues(isfinite(axisValues)));
        if numel(axisValues) >= 20
            lowerIndex = max(1, ceil(0.005 * numel(axisValues)));
            upperIndex = min(numel(axisValues), ...
                floor(0.995 * numel(axisValues)));
            lowerLimit = axisValues(lowerIndex);
            upperLimit = axisValues(upperIndex);
            if upperLimit > lowerLimit
                axisMargin = 0.08 * (upperLimit - lowerLimit);
                ylim(axisHandle, [lowerLimit - axisMargin, ...
                    upperLimit + axisMargin]);
            end
        end
        hold(axisHandle, "off");
        title(axisHandle, coefficientNames(coefficientNumber));
        xlabel(axisHandle, "时间 / s");
        ylabel(axisHandle, "系数");
        grid(axisHandle, "on");
        axisHandle.FontSize = 10;
        if coefficientNumber == 1
            legend(axisHandle, "Location", "best");
        end
    end
    title(layout, one.Name + " 六分量气动系数核对（仅有效空中样本）");
    exportgraphics(figureHandle, fullfile(aeroOutputDir, ...
        "aero_coefficients_" + one.Name + ".png"), "Resolution", 180);
    close(figureHandle);
end

writetable(allTimeseries, fullfile(aeroOutputDir, ...
    "coefficient_timeseries.csv"));

%% 4.1 残差随迎角分布
for sourceName = unique(allTimeseries.Source, "stable")'
    figureHandle = figure("Visible", "off", ...
        "Position", [100, 100, 1500, 900]);
    layout = tiledlayout(figureHandle, 3, 2, ...
        "TileSpacing", "compact", "Padding", "compact");
    sourceRows = allTimeseries.Source == sourceName & allTimeseries.Valid;
    for coefficientNumber = 1:6
        axisHandle = nexttile(layout);
        coefficientName = coefficientNames(coefficientNumber);
        residualValues = ...
            allTimeseries.(coefficientName + "_Residual");
        scatterRows = sourceRows & isfinite(allTimeseries.AlphaDeg) & ...
            isfinite(allTimeseries.TASMps) & isfinite(residualValues);
        scatter(axisHandle, allTimeseries.AlphaDeg(scatterRows), ...
            residualValues(scatterRows), ...
            5, allTimeseries.TASMps(scatterRows), "filled", ...
            "MarkerFaceAlpha", 0.20);
        residualForAxis = sort(residualValues(scatterRows));
        if numel(residualForAxis) >= 20
            lowerIndex = max(1, ceil(0.005 * numel(residualForAxis)));
            upperIndex = min(numel(residualForAxis), ...
                floor(0.995 * numel(residualForAxis)));
            lowerLimit = residualForAxis(lowerIndex);
            upperLimit = residualForAxis(upperIndex);
            if upperLimit > lowerLimit
                axisMargin = 0.08 * (upperLimit - lowerLimit);
                ylim(axisHandle, [lowerLimit - axisMargin, ...
                    upperLimit + axisMargin]);
            end
        end
        yline(axisHandle, 0, "--", "Color", [0.2, 0.2, 0.2]);
        title(axisHandle, coefficientName);
        xlabel(axisHandle, "\alpha / deg");
        ylabel(axisHandle, "反算 - 正向");
        grid(axisHandle, "on");
        axisHandle.FontSize = 10;
    end
    colorbarHandle = colorbar(nexttile(layout, 6));
    colorbarHandle.Label.String = "TAS / (m/s)";
    title(layout, sourceName + " 气动残差随迎角分布");
    exportgraphics(figureHandle, fullfile(aeroOutputDir, ...
        "residual_vs_alpha_" + sourceName + ".png"), ...
        "Resolution", 180);
    close(figureHandle);
end

%% 4.2 发动机闭合图
figureHandle = figure("Visible", "off", ...
    "Position", [100, 100, 1100, 650]);
layout = tiledlayout(figureHandle, 1, 1, ...
    "TileSpacing", "compact", "Padding", "compact");
for caseIndex = 1:numel(flightCases)
    axisHandle = nexttile(layout);
    one = flightCases(caseIndex);
    result = caseResults(caseIndex);
    plot(axisHandle, one.TimeS, result.RequiredThrustN, ...
        "Color", [0.1, 0.1, 0.1], "DisplayName", "动力学所需");
    hold(axisHandle, "on");
    plot(axisHandle, one.TimeS, result.ThrustN, ...
        "Color", caseColors(caseIndex, :), "DisplayName", "发动机表");
    if one.Source == "px4"
        plot(axisHandle, one.TimeS, one.PluginThrustN, "--", ...
            "Color", [0.45, 0.45, 0.45], ...
            "DisplayName", "Gazebo插件");
    end
    hold(axisHandle, "off");
    title(axisHandle, one.Name);
    xlabel(axisHandle, "时间 / s");
    ylabel(axisHandle, "推力 / N");
    grid(axisHandle, "on");
    axisHandle.FontSize = 10;
    legend(axisHandle, "Location", "best");
end
title(layout, "150 kg日志20_03_51发动机推力闭合");
exportgraphics(figureHandle, fullfile(aeroOutputDir, ...
    "engine_thrust_closure.png"), "Resolution", 180);
close(figureHandle);

%% 4.3 保存MAT和JSON摘要
% 不把四架次完整时序结构重复写入MAT文件。完整时序已经导出为CSV；
% 这里仅保存便于复核和制表的轻量摘要，避免Windows MATLAB通过WSL
% UNC路径写大型HDF5(-v7.3)文件时出现兼容性问题。
save(fullfile(aeroOutputDir, "px4_150kg_aero_results.mat"), ...
    "frameChecks", "coefficientMetrics", "pluginMetrics", ...
    "engineMetrics", "sensitivityMetrics", "inertiaVectorFRD", "-v7");

summary = struct;
summary.GeneratedBy = "MATLAB R2025a sequential scripts";
summary.MassKg = massKg;
summary.InertiaFRD = inertiaVectorFRD;
summary.CoordinateContract = ...
    "PX4 body FRD and navigation NED";
summary.XiangyiThrottleMapping = "not applicable";
summary.ReactionTorque = "-Mx_FRD, consistent with current V8 plugin";
summary.FilterWindowsS = filterWindowsS;
summary.FrameChecks = table2struct(frameChecks);
summary.CoefficientMetrics = table2struct(coefficientMetrics);
summary.PluginMetrics = table2struct(pluginMetrics);
summary.EngineMetrics = table2struct(engineMetrics);
summary.SensitivityMetrics = table2struct(sensitivityMetrics);
summary.Inputs = struct( ...
    "XiangyiCsv", xiangyiCsv, ...
    "PlanFile", planFile, ...
    "Px4UlogFiles", px4UlogFiles);
summary.Outputs = struct( ...
    "TimeseriesCsv", fullfile(aeroOutputDir, ...
    "coefficient_timeseries.csv"), ...
    "MetricsCsv", fullfile(aeroOutputDir, ...
    "coefficient_metrics.csv"));

jsonText = jsonencode(summary, "PrettyPrint", true);
fileID = fopen(fullfile(aeroOutputDir, ...
    "px4_150kg_aero_summary.json"), "w", "n", "UTF-8");
assert(fileID >= 0, "无法创建气动核对JSON摘要。");
fprintf(fileID, "%s\n", jsonText);
fclose(fileID);

disp("气动核对输出完成：")
disp(aeroOutputDir)
