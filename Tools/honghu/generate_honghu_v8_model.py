#!/usr/bin/env python3
"""Generate the Honghu Wing V8 SDF and solve base-link mass properties.

The PDF target is the assembled model target. Child-link mass properties are
subtracted with the parallel-axis theorem so the full neutral model is 150 kg,
has its CG at base_link, and has the requested Gazebo/FLU inertia tensor.
"""

from dataclasses import dataclass
from math import cos, sin
from pathlib import Path
from typing import Tuple

ROOT = Path(__file__).resolve().parents[2]
MODEL_DIR = ROOT / "simulation_models/models/honghu_wing_150kg_v8"
TARGET_MASS = 150.0
TARGET_INERTIA = [
    [25.86, 0.017, 3.520],
    [0.017, 39.14, -0.0019],
    [3.520, -0.0019, 59.12],
]


@dataclass(frozen=True)
class Part:
    mass: float
    position: Tuple[float, float, float]
    inertia: Tuple[float, float, float]


CONTROLS = [
    ("left_aileron", "left_aileron.dae", 0.020, (-0.01014, 1.881, 0.31284), (2.87e-5, 1.62e-5, 4.44e-5), (-0.01014, 1.881, 0.31284, 0.0, 0.0, -0.5788), (0, 1, 0), -0.523599, 0.523599),
    ("right_aileron", "right_aileron.dae", 0.020, (-0.01014, -1.881, 0.31284), (2.87e-5, 1.62e-5, 4.44e-5), (-0.01014, -1.881, 0.31284, 0.0, 0.0, 0.5792), (0, 1, 0), -0.523599, 0.523599),
    ("left_elevator", "left_elevator.dae", 0.020, (-0.7665, 0.39402, 0.41184), (3.07e-6, 1.27e-4, 1.29e-4), (-0.7065, 0.39402, 0.41184, -0.07, 0.0, 0.0), (0, 1, 0), -0.523599, 0.523599),
    ("right_elevator", "right_elevator.dae", 0.020, (-0.7665, -0.39402, 0.41184), (3.07e-6, 1.27e-4, 1.29e-4), (-0.7065, -0.39402, 0.41184, 0.07, 0.0, 0.0), (0, 1, 0), -0.523599, 0.523599),
    ("left_rudder", "left_rudder.dae", 0.010, (-0.89916, 1.4058, 0.1089), (2.06e-5, 1.77e-5, 3.02e-6), (-0.81916, 1.4058, 0.1089, 0.0, 0.0, 0.0), (0, 0, 1), -0.523599, 0.523599),
    ("right_rudder", "right_rudder.dae", 0.010, (-0.89916, -1.4058, 0.1089), (2.06e-5, 1.77e-5, 3.02e-6), (-0.81916, -1.4058, 0.1089, 0.0, 0.0, 0.0), (0, 0, 1), -0.523599, 0.523599),
    # Positive joint angle must be trailing-edge down, hence the reversed axis.
    ("left_canard", "left_canard.dae", 0.070, (1.30854, 0.40788, 0.03366), (2.97e-5, 1.31e-4, 1.57e-4), (1.30854, 0.40788, 0.03366, 0.096, 0.0, -0.368), (0, -1, 0), -0.872665, 0.261799),
    ("right_canard", "right_canard.dae", 0.070, (1.30854, -0.40788, 0.03366), (2.97e-5, 1.31e-4, 1.57e-4), (1.30854, -0.40788, 0.03366, -0.096, 0.0, 0.368), (0, -1, 0), -0.872665, 0.261799),
]

# V8 phase-1 uses a rigid wheel carrier: only wheel spin and nose steering are
# dynamic. Structural strut mass is absorbed into base_link by the assembly
# solver, keeping the complete model at the PDF mass / CG / inertia target.
GEAR_PARTS = [
    Part(0.20, (-0.291274, 0.524303, -0.4551), (0.000364, 0.000353, 0.000364)),
    Part(0.20, (-0.291274, -0.524303, -0.4551), (0.000364, 0.000353, 0.000364)),
    Part(0.05, (0.924852, 0.0, -0.4706), (5e-5, 5e-5, 5e-5)),
    Part(0.15, (0.924852, 0.0, -0.4706), (0.000304, 0.000145, 0.000304)),
]


def mat_add(a, b):
    return [[a[i][j] + b[i][j] for j in range(3)] for i in range(3)]


def mat_sub(a, b):
    return [[a[i][j] - b[i][j] for j in range(3)] for i in range(3)]


def parallel_axis(mass, r):
    x, y, z = r
    return [
        [mass * (y*y + z*z), -mass*x*y, -mass*x*z],
        [-mass*x*y, mass * (x*x + z*z), -mass*y*z],
        [-mass*x*z, -mass*y*z, mass * (x*x + y*y)],
    ]


def solve_base_properties():
    parts = [Part(row[2], row[3], row[4]) for row in CONTROLS] + GEAR_PARTS
    child_mass = sum(p.mass for p in parts)
    base_mass = TARGET_MASS - child_mass
    child_moment = [sum(p.mass * p.position[i] for p in parts) for i in range(3)]
    base_com = tuple(-value / base_mass for value in child_moment)
    child_inertia = [[0.0] * 3 for _ in range(3)]
    for part in parts:
        intrinsic = [
            [part.inertia[0], 0.0, 0.0],
            [0.0, part.inertia[1], 0.0],
            [0.0, 0.0, part.inertia[2]],
        ]
        child_inertia = mat_add(child_inertia, mat_add(intrinsic, parallel_axis(part.mass, part.position)))
    base_inertia = mat_sub(mat_sub(TARGET_INERTIA, child_inertia), parallel_axis(base_mass, base_com))
    return base_mass, base_com, base_inertia


def fmt_pose(values):
    return " ".join(f"{value:.9g}" for value in values)


def rotate_rpy(vector, roll, pitch, yaw):
    # SDF fixed-axis roll-pitch-yaw: Rz(yaw) Ry(pitch) Rx(roll).
    x, y, z = vector
    x, y, z = x, cos(roll)*y - sin(roll)*z, sin(roll)*y + cos(roll)*z
    x, y, z = cos(pitch)*x + sin(pitch)*z, y, -sin(pitch)*x + cos(pitch)*z
    return (cos(yaw)*x - sin(yaw)*y, sin(yaw)*x + cos(yaw)*y, z)


def inertia_xml(mass, position, diag):
    return f"""      <inertial>
        <pose>{fmt_pose((*position, 0, 0, 0))}</pose>
        <mass>{mass:.12g}</mass>
        <inertia>
          <ixx>{diag[0]:.12g}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{diag[1]:.12g}</iyy><iyz>0</iyz><izz>{diag[2]:.12g}</izz>
        </inertia>
      </inertial>"""


def control_xml(index, row):
    name, mesh, mass, com, diag, pose, axis, lower, upper = row
    return f"""
    <link name="{name}">
      <pose relative_to="base_link">0 0 0 0 0 0</pose>
      <gravity>true</gravity>
{inertia_xml(mass, com, diag)}
      <visual name="{name}_visual">
        <pose>1.57 0 0 0 0 0</pose>
        <geometry><mesh><uri>meshes/{mesh}</uri></mesh></geometry>
        <material><ambient>0.8 0.05 0.05 1</ambient><diffuse>0.8 0.05 0.05 1</diffuse></material>
      </visual>
    </link>
    <joint name="servo_{index}" type="revolute">
      <parent>base_link</parent><child>{name}</child>
      <pose relative_to="base_link">{fmt_pose(pose)}</pose>
      <axis>
        <xyz>{fmt_pose(axis)}</xyz>
        <limit><lower>{lower}</lower><upper>{upper}</upper><effort>400</effort><velocity>4</velocity></limit>
        <dynamics><damping>0.02</damping><friction>0.0001</friction></dynamics>
      </axis>
      <physics><ode><implicit_spring_damper>1</implicit_spring_damper></ode></physics>
    </joint>"""


def wheel_contact_xml(name, radius, length, mu, mu2):
    return f"""      <collision name="{name}">
        <pose>0 0 0 1.57079632679 0 0</pose>
        <geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry>
        <surface>
          <contact><ode><kp>2000000</kp><kd>20000</kd><max_vel>0.2</max_vel><min_depth>0.0005</min_depth></ode></contact>
          <!-- Rigid strut, real rolling wheel: the revolute spin joint supplies
               longitudinal rolling while lateral tire grip stabilizes tracking. -->
          <friction><ode><mu>{mu}</mu><mu2>{mu2}</mu2><fdir1>1 0 0</fdir1></ode></friction>
        </surface>
      </collision>"""

def main_gear_xml(side, y, visual_pose, mesh):
    wheel_joint = f"{side}_main_wheel_joint"
    wheel = f"{side}_main_wheel"
    return f"""
    <!-- Phase-1 rigid main gear: carrier fixed in base_link, wheel spins freely. -->
    <joint name="{wheel_joint}" type="revolute">
      <parent>base_link</parent><child>{wheel}</child>
      <pose relative_to="base_link">-0.291274 {y} -0.4551 0 0 0</pose>
      <axis><xyz>0 1 0</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit><dynamics><friction>0.02</friction></dynamics></axis>
    </joint>
    <link name="{wheel}">
      <pose relative_to="{wheel_joint}">0 0 0 0 0 0</pose>
      <gravity>true</gravity>
{inertia_xml(0.20, (0, 0, 0), (0.000364, 0.000353, 0.000364))}
      <visual name="wheel_visual"><pose>{visual_pose}</pose><geometry><mesh><uri>meshes/{mesh}</uri></mesh></geometry></visual>
{wheel_contact_xml(f"{side}_main_wheel_collision", "0.0594", "0.1060", "0.8", "2.0")}
    </link>"""


def nose_gear_xml():
    return f"""
    <!-- Rigid nose carrier with retained steering and free wheel spin. -->
    <joint name="nose_steering_joint" type="revolute">
      <parent>base_link</parent><child>nose_steering_fork</child>
      <pose relative_to="base_link">0.924852 0 -0.4706 0 0 0</pose>
      <axis><xyz>0 0 1</xyz><limit><lower>-0.523599</lower><upper>0.523599</upper><effort>300</effort><velocity>3</velocity></limit>
        <dynamics><damping>0.05</damping><friction>0.001</friction></dynamics></axis>
    </joint>
    <link name="nose_steering_fork">
      <pose relative_to="nose_steering_joint">0 0 0 0 0 0</pose>
      <gravity>true</gravity>
{inertia_xml(0.05, (0, 0, 0), (5e-5, 5e-5, 5e-5))}
    </link>
    <joint name="nose_wheel_spin_joint" type="revolute">
      <parent>nose_steering_fork</parent><child>nose_wheel</child>
      <pose relative_to="nose_steering_fork">0 0 0 0 0 0</pose>
      <axis><xyz>0 1 0</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit><dynamics><friction>0.02</friction></dynamics></axis>
    </joint>
    <link name="nose_wheel">
      <pose relative_to="nose_wheel_spin_joint">0 0 0 0 0 0</pose>
      <gravity>true</gravity>
{inertia_xml(0.15, (0, 0, 0), (0.000304, 0.000145, 0.000304))}
      <visual name="wheel_visual"><pose>0.645148 0 0.457844 0 0 0</pose><geometry><mesh><uri>meshes/steeringwheel.dae</uri></mesh></geometry></visual>
{wheel_contact_xml("nose_wheel_collision", "0.0439", "0.1363", "1.2", "3.0")}
    </link>"""


SENSORS = """
      <sensor name="imu_sensor" type="imu">
        <always_on>1</always_on><update_rate>250</update_rate>
        <imu>
          <angular_velocity>
            <x><noise type="gaussian"><mean>0</mean><stddev>1e-5</stddev></noise></x>
            <y><noise type="gaussian"><mean>0</mean><stddev>1e-5</stddev></noise></y>
            <z><noise type="gaussian"><mean>0</mean><stddev>1e-5</stddev></noise></z>
          </angular_velocity>
          <linear_acceleration>
            <x><noise type="gaussian"><mean>0</mean><stddev>5e-5</stddev></noise></x>
            <y><noise type="gaussian"><mean>0</mean><stddev>5e-5</stddev></noise></y>
            <z><noise type="gaussian"><mean>0</mean><stddev>5e-5</stddev></noise></z>
          </linear_acceleration>
        </imu>
      </sensor>
      <sensor name="air_pressure_sensor" type="air_pressure">
        <always_on>1</always_on><update_rate>50</update_rate>
        <air_pressure><pressure><noise type="gaussian"><mean>0</mean><stddev>0.01</stddev></noise></pressure></air_pressure>
      </sensor>
      <sensor name="magnetometer_sensor" type="magnetometer">
        <always_on>1</always_on><update_rate>100</update_rate>
        <magnetometer>
          <x><noise type="gaussian"><stddev>0.0001</stddev></noise></x>
          <y><noise type="gaussian"><stddev>0.0001</stddev></noise></y>
          <z><noise type="gaussian"><stddev>0.0001</stddev></noise></z>
        </magnetometer>
      </sensor>
      <sensor name="navsat_sensor" type="navsat"><always_on>1</always_on><update_rate>30</update_rate></sensor>"""


def generate():
    base_mass, base_com, base_i = solve_base_properties()
    control_blocks = "\n".join(control_xml(i, row) for i, row in enumerate(CONTROLS))
    controllers = "\n".join(
        f"""    <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">
      <joint_name>servo_{i}</joint_name><sub_topic>servo_{i}</sub_topic>
      <p_gain>2</p_gain><i_gain>0</i_gain><d_gain>0</d_gain><cmd_max>20</cmd_max><cmd_min>-20</cmd_min>
    </plugin>""" for i in range(8)
    )
    axes = []
    for i, row in enumerate(CONTROLS):
        pose, axis = row[5], row[6]
        base_axis = rotate_rpy(axis, pose[3], pose[4], pose[5])
        axes.append(f"      <axis_{i}_base>{fmt_pose(base_axis)}</axis_{i}_base>")

    sdf = f"""<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <!-- V8 coordinate contract: model/base_link is Gazebo FLU at assembled CG.
       PDF/PX4 FRD vectors convert with diag(1,-1,-1).
       Meshes are CAD/nose-origin assets translated +1.57 m into the CG frame. -->
  <model name="honghu_wing_150kg_v8">
    <pose>0 0 0.5145 0 0 0</pose>
    <static>false</static>
    <self_collide>false</self_collide>
    <link name="base_link">
      <gravity>true</gravity>
      <inertial>
        <pose>{fmt_pose((*base_com, 0, 0, 0))}</pose>
        <mass>{base_mass:.12g}</mass>
        <inertia>
          <ixx>{base_i[0][0]:.12g}</ixx><ixy>{base_i[0][1]:.12g}</ixy><ixz>{base_i[0][2]:.12g}</ixz>
          <iyy>{base_i[1][1]:.12g}</iyy><iyz>{base_i[1][2]:.12g}</iyz><izz>{base_i[2][2]:.12g}</izz>
        </inertia>
      </inertial>
      <collision name="belly_collision">
        <pose>0.3 0 -0.39 0 0 0</pose><geometry><box><size>1.3 1.0 0.08</size></box></geometry>
        <surface><contact><ode><kp>2000000</kp><kd>20000</kd><max_vel>0.2</max_vel><min_depth>0.0005</min_depth></ode></contact>
          <friction><ode><mu>0.4</mu><mu2>0.4</mu2></ode></friction></surface>
      </collision>
      <visual name="body_visual">
        <pose>1.57 0 0 0 0 0</pose><geometry><mesh><uri>meshes/body.dae</uri></mesh></geometry>
        <material><ambient>0.175 0.175 0.175 1</ambient><diffuse>0.175 0.175 0.175 1</diffuse></material>
      </visual>
      <visual name="propeller_visual">
        <pose>-1.23 0 0.12 0 1.57079632679 0</pose>
        <geometry><mesh><scale>0.33 0.33 0.33</scale><uri>meshes/propeller_ccw.dae</uri></mesh></geometry>
      </visual>
{SENSORS}
    </link>
{control_blocks}
{main_gear_xml("left", 0.524303, "1.861274 -0.524303 0.4551 0 0 0", "left_backwheel.dae")}
{main_gear_xml("right", -0.524303, "1.861274 0.524303 0.4551 0 0 0", "right_backwheel.dae")}
{nose_gear_xml()}
{controllers}
    <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">
      <joint_name>nose_steering_joint</joint_name><sub_topic>servo_8</sub_topic>
      <p_gain>5</p_gain><i_gain>0</i_gain><d_gain>0</d_gain><cmd_max>20</cmd_max><cmd_min>-20</cmd_min>
    </plugin>
    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher">
      {"".join(f"<joint_name>servo_{i}</joint_name>" for i in range(8))}
      <joint_name>left_main_wheel_joint</joint_name><joint_name>right_main_wheel_joint</joint_name>
      <joint_name>nose_steering_joint</joint_name><joint_name>nose_wheel_spin_joint</joint_name>
    </plugin>
    <plugin filename="libHonghuAeroV8.so" name="honghu::v8::HonghuAeroV8">
      <link_name>base_link</link_name><table_dir>model://honghu_wing_150kg_v8/aero_tables</table_dir>
      <area>2.42</area><span>3.96</span><mac>0.62</mac><reference_altitude_m>0</reference_altitude_m>
      {"".join(f"<joint_{i}>servo_{i}</joint_{i}>" for i in range(8))}
{chr(10).join(axes)}
    </plugin>
    <plugin filename="libHonghuPropulsionV8.so" name="honghu::v8::HonghuPropulsionV8">
      <link_name>base_link</link_name><table_dir>model://honghu_wing_150kg_v8/propulsion_tables</table_dir>
      <engine_point>-1.23 0 0.12</engine_point><thrust_down_deg>3</thrust_down_deg>
      <!-- Front-view CCW propeller: omega is +X, airframe reaction torque is -X. -->
      <propeller_rotation_sign>1</propeller_rotation_sign>
      <tau_up_s>0.5</tau_up_s><tau_down_s>0.3</tau_down_s><reference_altitude_m>0</reference_altitude_m>
    </plugin>
  </model>
</sdf>
"""
    MODEL_DIR.mkdir(parents=True, exist_ok=True)
    (MODEL_DIR / "model.sdf").write_text(sdf, encoding="utf-8")
    return base_mass, base_com, base_i


if __name__ == "__main__":
    mass, com, inertia = generate()
    print(f"generated {MODEL_DIR / 'model.sdf'}")
    print(f"base mass={mass:.9f} kg, base COM={com}")
    print("base inertia:")
    for row in inertia:
        print("  " + " ".join(f"{value:.9f}" for value in row))
