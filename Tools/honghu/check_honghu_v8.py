#!/usr/bin/env python3
"""Static acceptance checks for the Honghu Wing V8 model contract."""

import csv
import importlib.util
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

from audit_honghu_v8_aero import audit_model
from honghu_v8_aero_model import HonghuV8AeroModel

ROOT = Path(__file__).resolve().parents[2]
MODEL = ROOT / "simulation_models/models/honghu_wing_150kg_v8"
SDF = MODEL / "model.sdf"
WORLD = ROOT / "Tools/simulation/gz/worlds/honghu_v8.sdf"
AIRFRAME = ROOT / "ROMFS/px4fmu_common/init.d-posix/airframes/4028_gz_honghu_wing_150kg_v8"
GZ_INIT = ROOT / "ROMFS/px4fmu_common/init.d-posix/px4-rc.gzsim"
GENERATOR = ROOT / "Tools/honghu/generate_honghu_v8_model.py"
PROVENANCE = MODEL / "data_provenance.yaml"
README = MODEL / "README.md"
TOL = 1e-8


def fail(message):
    raise AssertionError(message)


def close(actual, expected, tol=TOL, label="value"):
    if not math.isclose(actual, expected, rel_tol=tol, abs_tol=tol):
        fail(f"{label}: expected {expected}, got {actual}")


def load_generator():
    spec = importlib.util.spec_from_file_location("honghu_v8_generator", GENERATOR)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def csv_grid(path):
    rows = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.reader(line for line in handle if line.strip() and not line.lstrip().startswith("#"))
        header = next(reader)
        columns = [float(value) for value in header[1:]]
        for row in reader:
            rows.append((float(row[0]), [float(value) for value in row[1:]]))
    return columns, rows


def interp(path, row_query, column_query):
    columns, rows = csv_grid(path)
    row_axis = [item[0] for item in rows]

    def bracket(axis, value):
        value = max(axis[0], min(axis[-1], value))
        for index in range(1, len(axis)):
            if value <= axis[index]:
                return index - 1, index, value
        return len(axis) - 2, len(axis) - 1, value

    r0, r1, rq = bracket(row_axis, row_query)
    c0, c1, cq = bracket(columns, column_query)
    tr = (rq - row_axis[r0]) / (row_axis[r1] - row_axis[r0])
    tc = (cq - columns[c0]) / (columns[c1] - columns[c0])
    a = rows[r0][1][c0] + tc * (rows[r0][1][c1] - rows[r0][1][c0])
    b = rows[r1][1][c0] + tc * (rows[r1][1][c1] - rows[r1][1][c0])
    return a + tr * (b - a)


def check_mass_and_inertia(root, gen):
    mass, base_com, inertia = gen.solve_base_properties()
    base = root.find("./model/link[@name='base_link']")
    close(float(base.findtext("inertial/mass")), mass, label="base mass")
    pose = [float(value) for value in base.findtext("inertial/pose").split()]
    for i, expected in enumerate(base_com):
        close(pose[i], expected, tol=2e-9, label=f"base COM[{i}]")
    names = (("ixx", 0, 0), ("ixy", 0, 1), ("ixz", 0, 2),
             ("iyy", 1, 1), ("iyz", 1, 2), ("izz", 2, 2))
    for tag, i, j in names:
        close(float(base.findtext(f"inertial/inertia/{tag}")), inertia[i][j], tol=2e-9, label=tag)
    total_mass = sum(float(link.findtext("inertial/mass")) for link in root.findall("./model/link"))
    close(total_mass, 150.0, label="assembled mass")
    child_moment = [
        sum(part.mass * part.position[i] for part in
            [gen.Part(row[2], row[3], row[4]) for row in gen.CONTROLS] + gen.GEAR_PARTS)
        for i in range(3)
    ]
    for i in range(3):
        close(mass * base_com[i] + child_moment[i], 0.0, tol=2e-9, label=f"assembled CG moment[{i}]")


def check_sdf_contract(root, gen):
    model = root.find("./model")
    if model.attrib.get("name") != "honghu_wing_150kg_v8":
        fail("wrong model name")
    close(float(model.findtext("pose").split()[2]), 0.5145, label="spawn height")

    expected_axes = []
    for i, row in enumerate(gen.CONTROLS):
        joint = model.find(f"joint[@name='servo_{i}']")
        axis = tuple(float(value) for value in joint.findtext("axis/xyz").split())
        if axis != tuple(float(value) for value in row[6]):
            fail(f"servo_{i} axis mismatch")
        limits = joint.find("axis/limit")
        close(float(limits.findtext("lower")), row[7], tol=1e-6, label=f"servo_{i} lower")
        close(float(limits.findtext("upper")), row[8], tol=1e-6, label=f"servo_{i} upper")
        close(float(joint.findtext("axis/dynamics/damping")), 0.02, label=f"servo_{i} damping")
        close(float(joint.findtext("axis/dynamics/friction")), 0.0001, label=f"servo_{i} friction")
        pose = row[5]
        expected_axes.append(gen.rotate_rpy(row[6], pose[3], pose[4], pose[5]))

    controllers = {
        plugin.findtext("joint_name"): plugin
        for plugin in model.findall("plugin")
        if plugin.attrib.get("name") == "gz::sim::systems::JointPositionController"
    }
    for i in range(8):
        controller = controllers[f"servo_{i}"]
        close(float(controller.findtext("p_gain")), 2.0, label=f"servo_{i} p_gain")
        close(float(controller.findtext("d_gain")), 0.0, label=f"servo_{i} d_gain")
        close(float(controller.findtext("cmd_max")), 20.0, label=f"servo_{i} cmd_max")
    nose_joint = model.find("joint[@name='nose_steering_joint']")
    close(float(nose_joint.findtext("axis/dynamics/damping")), 0.05, label="nose steering damping")
    close(float(nose_joint.findtext("axis/dynamics/friction")), 0.001, label="nose steering friction")
    nose_controller = controllers["nose_steering_joint"]
    close(float(nose_controller.findtext("p_gain")), 1000.0, label="nose steering p_gain")
    close(float(nose_controller.findtext("i_gain")), 500.0, label="nose steering i_gain")
    close(float(nose_controller.findtext("d_gain")), 30.0, label="nose steering d_gain")
    close(float(nose_controller.findtext("i_max")), 80.0, label="nose steering i_max")
    close(float(nose_controller.findtext("i_min")), -80.0, label="nose steering i_min")
    close(float(nose_controller.findtext("cmd_max")), 200.0, label="nose steering cmd_max")
    close(float(nose_controller.findtext("cmd_min")), -200.0, label="nose steering cmd_min")
    if float(nose_controller.findtext("cmd_max")) >= float(nose_joint.findtext("axis/limit/effort")):
        fail("nose steering controller command must remain below the joint effort limit")

    aero = model.find("plugin[@name='honghu::v8::HonghuAeroV8']")
    for i, expected in enumerate(expected_axes):
        actual = [float(value) for value in aero.findtext(f"axis_{i}_base").split()]
        for j in range(3):
            close(actual[j], expected[j], tol=2e-8, label=f"axis_{i}_base[{j}]")

    # Phase-1 ground model is deliberately rigid: no suspension DOF can add
    # energy or backend-dependent spring behavior. Wheel spin and nose steering
    # remain dynamic and are attached at the PDF-derived gear coordinates.
    if model.findall("joint[@type='prismatic']"):
        fail("rigid gear baseline must not contain prismatic suspension joints")
    gear_contract = {
        "left_main_wheel_joint": ("base_link", "left_main_wheel", (-0.291274, 0.524303, -0.4551)),
        "right_main_wheel_joint": ("base_link", "right_main_wheel", (-0.291274, -0.524303, -0.4551)),
        "nose_steering_joint": ("base_link", "nose_steering_fork", (0.924852, 0.0, -0.4706)),
        "nose_wheel_spin_joint": ("nose_steering_fork", "nose_wheel", (0.0, 0.0, 0.0)),
    }
    for name, (parent, child, position) in gear_contract.items():
        joint = model.find(f"joint[@name='{name}']")
        if joint is None or joint.findtext("parent") != parent or joint.findtext("child") != child:
            fail(f"{name} rigid attachment mismatch")
        pose = tuple(float(value) for value in joint.findtext("pose").split()[:3])
        for i in range(3):
            close(pose[i], position[i], label=f"{name} position[{i}]")

    wheel_contract = {
        "left_main_wheel": ("left_main_wheel_collision", 0.0594, 0.1060, 0.8, 2.0),
        "right_main_wheel": ("right_main_wheel_collision", 0.0594, 0.1060, 0.8, 2.0),
        "nose_wheel": ("nose_wheel_collision", 0.0439, 0.1363, 1.2, 3.0),
    }
    for link_name, (collision_name, radius, length, mu, mu2) in wheel_contract.items():
        link = model.find(f"link[@name='{link_name}']")
        collision = link.find(f"collision[@name='{collision_name}']") if link is not None else None
        if collision is None:
            fail(f"{link_name} must carry its own rolling cylinder collision")
        if collision.find("geometry/cylinder") is None:
            fail(f"{collision_name} must be a cylinder")
        close(float(collision.findtext("geometry/cylinder/radius")), radius,
              label=f"{collision_name} radius")
        close(float(collision.findtext("geometry/cylinder/length")), length,
              label=f"{collision_name} length")
        pose = tuple(float(value) for value in collision.findtext("pose").split())
        close(pose[3], 1.57079632679, label=f"{collision_name} cylinder axis rotation")
        close(float(collision.findtext("surface/friction/ode/mu")), mu,
              label=f"{collision_name} longitudinal friction")
        close(float(collision.findtext("surface/friction/ode/mu2")), mu2,
              label=f"{collision_name} lateral friction")
        close(float(collision.findtext("surface/contact/ode/kp")), 2.0e6,
              label=f"{collision_name} contact kp")
        close(float(collision.findtext("surface/contact/ode/kd")), 2.0e4,
              label=f"{collision_name} contact kd")
        close(float(collision.findtext("surface/contact/ode/max_vel")), 0.2,
              label=f"{collision_name} max correction velocity")
        close(float(collision.findtext("surface/contact/ode/min_depth")), 0.0005,
              label=f"{collision_name} min_depth")
    forbidden_contacts = {
        "left_main_ground_contact", "right_main_ground_contact", "nose_ground_contact"
    }
    if any(collision.get("name") in forbidden_contacts for collision in model.findall(".//collision")):
        fail("V8 rolling gear must not retain body/fork skid contacts")

    propulsion = model.find("plugin[@name='honghu::v8::HonghuPropulsionV8']")
    if propulsion.find("command_topic") is not None:
        fail("propulsion topic must derive from the runtime model instance")
    if propulsion.findtext("engine_point") != "-1.23 0 0.12":
        fail("engine point mismatch")
    close(float(propulsion.findtext("thrust_down_deg")), 3.0, label="thrust angle")
    close(float(propulsion.findtext("propeller_rotation_sign")), 1.0, label="propeller +X rotation sign")
    source = (ROOT / "src/modules/simulation/gz_plugins/honghu_v8/HonghuPropulsionV8.cpp").read_text()
    if "\"/model/\"+model_name+\"/honghu_v8/motor_command\"" not in source:
        fail("propulsion must subscribe to the model-scoped ESC mirror")
    esc_bridge = (ROOT / "src/modules/simulation/gz_bridge/GZMixingInterfaceESC.cpp").read_text()
    if "\"/model/\" + model_name + \"/honghu_v8/motor_command\"" not in esc_bridge:
        fail("ESC bridge model-scoped mirror is missing")
    if "\"/\" + model_name + \"/command/motor_speed\"" not in esc_bridge:
        fail("ESC bridge legacy root topic must be preserved")
    if "_node.Unsubscribe(_command_topic)" not in source or "kCommandMissing" not in source:
        fail("propulsion command discovery refresh / missing flag is absent")
    if "-_propeller_rotation_sign*sample.torque_nm" not in source:
        fail("engine torque must be applied as airframe reaction torque")


def check_control_signs():
    tables = MODEL / "aero_tables/control_tables"
    da_pos = interp(tables / "aileron_Cl.csv", 0, 1) * 1
    da_neg = interp(tables / "aileron_Cl.csv", 0, -1) * -1
    de_pos = interp(tables / "elevator_Cm.csv", 0, 1) * 1
    de_neg = interp(tables / "elevator_Cm.csv", 0, -1) * -1
    dr_pos = interp(tables / "rudder_Cn.csv", 0, 0) * 1
    dr_neg = interp(tables / "rudder_Cn.csv", 0, 0) * -1
    dc_pos = interp(tables / "canard_Cm.csv", 0, 1) * 1
    dc_neg = interp(tables / "canard_Cm.csv", 0, -1) * -1

    # FRD coefficient moments then the single FLU conversion diag(1,-1,-1).
    cases = {
        "aileron+": (da_pos, da_pos),
        "aileron-": (da_neg, da_neg),
        "elevator+": (de_pos, -de_pos),
        "elevator-": (de_neg, -de_neg),
        "rudder+": (dr_pos, -dr_pos),
        "rudder-": (dr_neg, -dr_neg),
        "canard+": (dc_pos, -dc_pos),
        "canard-": (dc_neg, -dc_neg),
    }
    if not (cases["aileron+"][0] > 0 and cases["aileron+"][1] > 0 and cases["aileron-"][1] < 0):
        fail("aileron sign contract failed")
    if not (cases["elevator+"][0] > 0 and cases["elevator+"][1] < 0 and cases["elevator-"][1] > 0):
        fail("elevator sign contract failed")
    if not (cases["rudder+"][0] > 0 and cases["rudder+"][1] < 0 and cases["rudder-"][1] > 0):
        fail("rudder sign contract failed")
    if not (cases["canard+"][0] > 0 and cases["canard+"][1] < 0 and cases["canard-"][1] > 0):
        fail("canard sign contract failed")

    # Positive PX4 moment commands must create positive document deflections.
    theta_roll = (-1, 1, 0, 0, 0, 0, 0, 0)
    theta_pitch = (0, 0, 1, 1, 0, 0, 0, 0)
    theta_yaw = (0, 0, 0, 0, 1, 1, 0, 0)
    theta_canard = (0, 0, 0, 0, 0, 0, 1, 1)
    close(0.5 * (-theta_roll[0] + theta_roll[1]), 1, label="PX4 roll to delta_a")
    close(0.5 * (theta_pitch[2] + theta_pitch[3]), 1, label="PX4 pitch to delta_e")
    close(0.5 * (theta_yaw[4] + theta_yaw[5]), 1, label="PX4 yaw to delta_r")
    close(0.5 * (theta_canard[6] + theta_canard[7]), 1, label="PX4 canard to delta_c")


def check_tables():
    static_dir = MODEL / "aero_tables"
    for path in static_dir.rglob("*.csv"):
        columns, rows = csv_grid(path)
        if not columns or len(rows) < 1:
            fail(f"empty table {path}")
        for value in columns + [v for _, values in rows for v in values]:
            if not math.isfinite(value):
                fail(f"non-finite value in {path}")
    close(interp(static_dir / "CL.csv", 18, 0), 1.3813, tol=1e-7, label="CL(18,0)")
    close(interp(static_dir / "CL.csv", 20, 0), 1.4057, tol=1e-7, label="CL(20,0)")
    close(interp(static_dir / "CD.csv", 18, 0), 0.2556, tol=1e-7, label="CD(18,0)")
    close(interp(static_dir / "Cm.csv", 20, 0), -0.0994, tol=1e-7, label="Cm(20,0)")

    with (MODEL / "propulsion_tables/propeller.csv").open(encoding="utf-8") as handle:
        prop_rows = list(csv.DictReader(handle))
    with (MODEL / "propulsion_tables/fuel.csv").open(encoding="utf-8") as handle:
        fuel_rows = list(csv.DictReader(handle))
    if len(prop_rows) != 72:
        fail(f"expected 72 propulsion rows, got {len(prop_rows)}")
    if len(fuel_rows) != 144:
        fail(f"expected 144 fuel rows, got {len(fuel_rows)}")
    negative_prop = [
        row for row in prop_rows
        if float(row["thrust_kgf"]) < 0 or float(row["torque_Nm"]) < 0
    ]
    negative_keys = {
        (float(row["altitude_m"]), float(row["throttle_pct"]), float(row["rpm"]), float(row["airspeed_mps"]))
        for row in negative_prop
    }
    expected_negative = {(0.0, 65.0, 3000.0, 50.0), (2000.0, 70.0, 3000.0, 50.0)}
    if negative_keys != expected_negative:
        fail(f"unexpected negative propulsion cases: {negative_keys}")
    corrected = [row for row in fuel_rows if row["quality"] == "imputed_from_16_25_pct"]
    if len(corrected) != 1:
        fail("expected one explicitly marked corrected fuel datum")


def check_servo_bridge():
    source = (ROOT / "src/modules/simulation/gz_bridge/GZMixingInterfaceServo.cpp").read_text()
    yaml = (ROOT / "src/modules/simulation/gz_bridge/module.yaml").read_text()
    if "if (_servo_zero_mapping.get())" not in source:
        fail("piecewise servo zero mapping branch is missing")
    legacy = "output = _angle_min_rad[i] + _angular_range_rad[i]"
    if legacy not in source:
        fail("legacy minimum-to-maximum mapping branch changed or missing")
    if "SIM_GZ_SV_ZMAP:" not in yaml or "default: false" not in yaml:
        fail("legacy mapping must remain the default")

    def legacy_map(command, minimum, maximum):
        return minimum + (maximum - minimum) * command

    for command in (0.0, 0.1, 0.5, 0.9, 1.0):
        close(legacy_map(command, -30.0, 30.0), -30.0 + 60.0 * command,
              label=f"legacy mapping at {command}")

    def zero_map(normalized, minimum, zero, maximum):
        if normalized <= 0.0:
            return zero + normalized * (zero - minimum)
        return zero + normalized * (maximum - zero)

    for normalized, expected in ((-1.0, -50.0), (0.0, 0.0), (1.0, 15.0)):
        close(zero_map(normalized, -50.0, 0.0, 15.0), expected,
              label=f"asymmetric canard mapping at {normalized}")
    for normalized, expected in ((-1.0, 30.0), (0.0, 0.0), (1.0, -30.0)):
        close(zero_map(normalized, 30.0, 0.0, -30.0), expected,
              label=f"reversed nose-wheel mapping at {normalized}")


def check_airframe():
    text = AIRFRAME.read_text(encoding="utf-8")
    required = (
        "PX4_GZ_MAX_STEP_SIZE=${PX4_GZ_MAX_STEP_SIZE:=0.002}",
        "PX4_GZ_HOME_LAT=${PX4_GZ_HOME_LAT:=28.5712315}",
        "PX4_GZ_HOME_LON=${PX4_GZ_HOME_LON:=121.5759172}",
        "PX4_GZ_HOME_ALT=${PX4_GZ_HOME_ALT:=0}",
        "PX4_GZ_HOME_HEADING=${PX4_GZ_HOME_HEADING:=0}",
        "PX4_GZ_SET_HOME_COORDINATES=${PX4_GZ_SET_HOME_COORDINATES:=1}",
        "PX4_GZ_MODEL_POSE=${PX4_GZ_MODEL_POSE:=0,0,0.5145,0,0,1.1349764}",
        "param set SIM_GZ_SV_ZMAP 1",
        "param set SIM_GZ_EC_MIN1 0",
        "param set SIM_GZ_EC_MAX1 1000",
        "param set FW_THR_MAX 1.0",
        "param set RWTO_MAX_THR 1.0",
        "param set FW_W_TC 2.50",
        "param set FW_WR_P 0.25",
        "param set FW_WR_FF 0.08",
        "param set FW_W_RMAX 20.0",
        "param set FW_W_GSPD_SC 5.0",
        "param set RWTO_TAXI_XTK_P 0.010",
        "param set RWTO_TAXI_XMAX 20.0",
        "param set RWTO_TAXI_YRMAX 15.0",
        "param set SIM_GZ_SV_MINA9 30",
        "param set SIM_GZ_SV_ZEROA9 0",
        "param set SIM_GZ_SV_MAXA9 -30",
        "param set FW_CANARD_NEUT   0.5",
        "param set FW_CANARD_TO     0.266667",
        "param set FW_CANARD_BRK    1.0",
        "param set RWTO_WHEEL_HGT 0.20",
        "param set FW_P_LIM_MAX 8",
        "param set FW_PR_FF_RWTO 6.6",
        "param set FW_PR_RWTO_Q 2.0",
        "param set FW_P_RMAX_POS 6",
        "param set TRIM_PITCH 0.03",
        "param set FW_PR_P 0.40",
        "param set FW_PR_D 0.10",
        "param set FW_PR_FF 0.75",
        "param set FW_PR_I 0.04",
        "param set FW_PR_IMAX 0.12",
        "param set FW_P_TC 0.8",
        "param set FW_P_RMAX_NEG 10",
        "param set FW_T_I_GAIN_PIT 0.05",
        "param set FW_T_PTCH_DAMP 0.15",
        "param set FW_T_ALT_TC 3.5",
        "param set FW_T_RLL2THR 20",
        "param set FW_RR_P 0.26",
        "param set FW_RR_FF 1.45",
        "param set FW_RR_I 0.05",
        "param set FW_RR_D 0.06",
        "param set FW_R_TC 0.65",
        "param set FW_R_LIM 30",
        "param set FW_R_RMAX 20",
        "param set FW_PN_R_SLEW_MAX 20",
        "param set NPFG_PERIOD 20.0",
        "param set NPFG_DAMPING 0.80",
        "param set NPFG_ROLL_TC 1.30",
        "param set NPFG_SW_DST_MLT 0.32",
        "param set NAV_ACC_RAD 250",
        "param set RWTO_DIR_MIN 50",
        "param set-default CA_SV_CS6_TYPE  19",
        "param set-default CA_SV_CS7_TYPE  20",
    )
    for item in required:
        if item not in text:
            fail(f"missing airframe contract: {item}")
    for i in range(6):
        if f"param set-default CA_SV_CS{i}_TYPE  12" not in text:
            fail(f"V8 paired conventional surface {i} must use explicit Custom effectiveness")
    for item in ("CA_SV_CS6_TRQ_P", "CA_SV_CS7_TRQ_P"):
        if item in text:
            fail(f"state-machine canard must not enter pitch allocation: {item}")
    for i in range(1, 7):
        for item in (f"MINA{i} -30", f"ZEROA{i} 0", f"MAXA{i} 30"):
            if item not in text:
                fail(f"missing servo travel {item}")
    for i in (7, 8):
        for item in (f"MINA{i} -50", f"ZEROA{i} 0", f"MAXA{i} 15"):
            if item not in text:
                fail(f"missing canard travel {item}")


def check_geographic_origin():
    world = ET.parse(WORLD).getroot()
    close(float(world.findtext("./world/spherical_coordinates/latitude_deg")),
          28.5712315, label="V8 world latitude")
    close(float(world.findtext("./world/spherical_coordinates/longitude_deg")),
          121.5759172, label="V8 world longitude")
    close(float(world.findtext("./world/spherical_coordinates/elevation")),
          0.0, label="V8 world elevation")
    magnetic_field = [float(value) for value in world.findtext("./world/magnetic_field").split()]
    expected_field = (6e-06, 2.3e-05, -4.2e-05)
    for index, expected in enumerate(expected_field):
        close(magnetic_field[index], expected, tol=1e-12,
              label=f"V8 world Gazebo-8-compatible magnetic field[{index}]")
    plane_sizes = [
        tuple(float(value) for value in element.text.split())
        for element in (
            world.findall("./world/model[@name='ground_plane']/link/collision/geometry/plane/size")
            + world.findall("./world/model[@name='ground_plane']/link/visual/geometry/plane/size")
        )
    ]
    if plane_sizes != [(30000.0, 30000.0), (30000.0, 30000.0)]:
        fail(f"V8 world ground size mismatch: {plane_sizes}")

    gz_init = GZ_INIT.read_text(encoding="utf-8")
    for item in ("/set_spherical_coordinates", "gz.msgs.SphericalCoordinates",
                 "latitude_deg: ${PX4_GZ_HOME_LAT}",
                 "longitude_deg: ${PX4_GZ_HOME_LON}",
                 "PX4_GZ_SET_HOME_COORDINATES:-1",
                 "orientation: { x: ${quat_x}",
                 "Spawning model at pose"):
        if item not in gz_init:
            fail(f"missing runtime geographic-origin contract: {item}")

    provenance = PROVENANCE.read_text(encoding="utf-8")
    if "rate_D_air_0.10" not in provenance:
        fail("data provenance does not match the validated pitch D gain")
    readme = README.read_text(encoding="utf-8")
    for item in ("control-source clamp=16", "derived static data=32", "30000 x 30000 m"):
        if item not in readme:
            fail(f"README is missing current V8 environment contract: {item}")


def main():
    gen = load_generator()
    root = ET.parse(SDF).getroot()
    check_mass_and_inertia(root, gen)
    check_sdf_contract(root, gen)
    check_control_signs()
    check_tables()
    aero_report = audit_model(HonghuV8AeroModel())
    check_servo_bridge()
    check_airframe()
    check_geographic_origin()
    print("Honghu V8 static contract: PASS")
    print("  mass=150 kg, CG=base_link, target FLU inertia closed")
    print("  control signs: positive delta_doc produces the required FRD/FLU moments")
    print("  rigid rolling-wheel contact, stable joint-controller parameters, 4028 and legacy bridge mapping verified")
    print("  WGS84 origin, Gazebo-8-compatible magnetic field, 30 km ground and mission-aligned spawn verified")
    print(f"  aerodynamic truth model: PASS ({aero_report['checks']} table/sign/continuity/trim checks)")


if __name__ == "__main__":
    main()
