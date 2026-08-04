%% 鸿鹄翼V8 100 kg当前代表架次与翔仪闭环对比
% 只读取已经存在的ULog，不启动或修改仿真模型。

clearvars;
close all;
clc;

scriptDir = fileparts(mfilename("fullpath"));
if ispc
    flyHome = "\\wsl.localhost\Ubuntu-22.04\home\fly";
else
    flyHome = "/home/fly";
end
repoRoot = fullfile(flyHome, "PX4-Autopilot-NewAero");

px4UlogFiles = [ ...
    fullfile(repoRoot, "build", "px4_sitl_default", "rootfs", "log", ...
        "2026-07-31", "04_48_31.ulg")];
outputRoot = fullfile(repoRoot, "analysis_outputs", ...
    "honghu_v8_xiangyi_latest_20260731", "matlab_current_final_closed_loop");

run(fullfile(scriptDir, "step_01_config_and_tables.m"));
run(fullfile(scriptDir, "step_02_import_and_normalize_logs.m"));
run(fullfile(scriptDir, "step_03_compute_aero_coefficients.m"));
run(fullfile(scriptDir, "step_05_compare_closed_loop.m"));

run(fullfile(scriptDir, "step_06_export_typical_parameters.m"));
disp("============================================================")
disp("最新100 kg闭环图与典型参数表已完成：")
disp(closedLoopOutputDir)
