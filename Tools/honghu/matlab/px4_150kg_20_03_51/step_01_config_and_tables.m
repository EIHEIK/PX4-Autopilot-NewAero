%% 1. 路径、物理常量、输入文件和查表数据
% 所有坐标、惯量和力矩计算统一使用PDF/PX4 FRD机体系。

if ispc
    flyHome = "\\wsl.localhost\Ubuntu-22.04\home\fly";
else
    flyHome = "/home/fly";
end

repoRoot = fullfile(flyHome, "PX4-Autopilot-NewAero");
referenceRoot = fullfile(flyHome, "px4_reference_docs", "current");
xiangyiCsv = fullfile(referenceRoot, "翔仪飞控仿真结果.csv");
planFile = fullfile(referenceRoot, "模仿XY航线规划.plan");

if ~exist("px4UlogFiles", "var")
    px4UlogFiles = [ ...
        fullfile(repoRoot, "build", "px4_sitl_default", "rootfs", "log", ...
            "2026-07-30", "20_03_51.ulg")];
end

if ~exist("outputRoot", "var")
    outputRoot = fullfile(repoRoot, "analysis_outputs", ...
        "honghu_v8_150kg_20_03_51_matlab");
end
aeroOutputDir = fullfile(outputRoot, "aero");
closedLoopOutputDir = fullfile(outputRoot, "closed_loop");
if ~isfolder(aeroOutputDir)
    mkdir(aeroOutputDir);
end
if ~isfolder(closedLoopOutputDir)
    mkdir(closedLoopOutputDir);
end

includeXiangyi = false;
assert(all(isfile(px4UlogFiles)), ...
    "找不到指定的150 kg PX4 ULog：20_03_51.ulg");

massKg = 150.0;
gravityMps2 = 9.8;
areaM2 = 2.42;
spanM = 3.96;
macM = 0.62;
sampleTimeS = 0.05;
groundBaseZM = 0.5145;
thrustDownRad = deg2rad(3.0);
enginePointFRDM = [-1.23, 0.0, -0.12];
reactionTorqueSignFRD = -1.0;
coefficientNames = ["CL", "CD", "CY", "Cl", "Cm", "Cn"];
phaseNames = ["all", "climb", "level", "turn", "descent"];
filterWindowsS = [0.25, 0.50, 1.00];
bootstrapCount = 500;
bootstrapSeed = 20260731;
rng(bootstrapSeed, "twister");

inertia73FRD = [25.33, 30.81, 50.98, -0.021, -2.592, -0.0002];
inertia150FRD = [25.86, 39.14, 59.12, -0.017, -3.520, -0.0019];
inertiaVectorFRD = inertia150FRD;
inertiaFRD = [ ...
    inertiaVectorFRD(1), inertiaVectorFRD(4), inertiaVectorFRD(5); ...
    inertiaVectorFRD(4), inertiaVectorFRD(2), inertiaVectorFRD(6); ...
    inertiaVectorFRD(5), inertiaVectorFRD(6), inertiaVectorFRD(3)];

aeroTableDir = fullfile(repoRoot, "simulation_models", "models", ...
    "honghu_wing_150kg_v8", "aero_tables");
propellerFile = fullfile(repoRoot, "simulation_models", "models", ...
    "honghu_wing_150kg_v8", "propulsion_tables", "propeller.csv");

staticNames = coefficientNames;
staticGrid = struct;
for tableNumber = 1:numel(staticNames)
    tableFile = fullfile(aeroTableDir, staticNames(tableNumber) + ".csv");
    tableCell = readcell(tableFile, "Delimiter", ",", "CommentStyle", "#");
    staticGrid.(staticNames(tableNumber)).beta = ...
        cell2mat(tableCell(1, 2:end));
    staticGrid.(staticNames(tableNumber)).alpha = ...
        cell2mat(tableCell(2:end, 1));
    staticGrid.(staticNames(tableNumber)).value = ...
        cell2mat(tableCell(2:end, 2:end));
end

controlNames = [ ...
    "aileron_CD", "aileron_CY", "aileron_Cl", "aileron_Cn", ...
    "elevator_CL", "elevator_CD", "elevator_Cm", ...
    "rudder_CD", "rudder_CY", "rudder_Cl", "rudder_Cn", ...
    "canard_CL", "canard_CD", "canard_Cm"];
controlGrid = struct;
for tableNumber = 1:numel(controlNames)
    tableFile = fullfile(aeroTableDir, "control_tables", ...
        controlNames(tableNumber) + ".csv");
    tableCell = readcell(tableFile, "Delimiter", ",", "CommentStyle", "#");
    controlGrid.(controlNames(tableNumber)).column = ...
        cell2mat(tableCell(1, 2:end));
    controlGrid.(controlNames(tableNumber)).alpha = ...
        cell2mat(tableCell(2:end, 1));
    controlGrid.(controlNames(tableNumber)).value = ...
        cell2mat(tableCell(2:end, 2:end));
end

propellerTable = readtable(propellerFile, "VariableNamingRule", "preserve");
propellerAltitudes = unique(propellerTable.altitude_m);

disp("统一核对配置：")
disp(table(massKg, areaM2, spanM, macM, ...
    'VariableNames', {'MassKg', 'AreaM2', 'SpanM', 'MacM'}))
disp("150 kg FRD惯量 [Ixx Iyy Izz Ixy Ixz Iyz]：")
disp(inertiaVectorFRD)
disp("PX4分析ULog：")
disp(px4UlogFiles)
