%% 鸿鹄翼 V8 150 kg日志20_03_51气动模型核对总入口
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

disp("============================================================")
disp("鸿鹄翼 V8 150 kg日志20_03_51气动核对已完成。")
disp("输出目录：")
disp(outputRoot)
