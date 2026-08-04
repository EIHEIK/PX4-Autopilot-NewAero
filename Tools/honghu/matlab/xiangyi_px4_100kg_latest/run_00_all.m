%% 鸿鹄翼 V8 100 kg 与翔仪双数据源统一核对总入口
% 本目录内全部文件都是顺序执行脚本，不包含自定义函数。
% 可以直接运行本文件，也可以在编辑器中按顺序执行各脚本。

clearvars;
close all;
clc;

scriptDir = fileparts(mfilename("fullpath"));
run(fullfile(scriptDir, "step_01_config_and_tables.m"));
run(fullfile(scriptDir, "step_02_import_and_normalize_logs.m"));
run(fullfile(scriptDir, "step_03_compute_aero_coefficients.m"));
run(fullfile(scriptDir, "step_04_export_aero_comparison.m"));
run(fullfile(scriptDir, "step_05_compare_closed_loop.m"));

disp("============================================================")
disp("鸿鹄翼 V8 100 kg 与翔仪统一MATLAB核对已完成。")
disp("输出目录：")
disp(outputRoot)
