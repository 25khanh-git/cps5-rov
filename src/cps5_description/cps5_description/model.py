"""Sinh SDF cho robot CPS 5 từ file YAML tham số.

Mọi con số vật lý nằm trong config/cps5_vehicle.yaml. File này chỉ "dịch"
các con số đó sang SDF 1.11 cho Gazebo Harmonic, và tính các đại lượng suy ra
(thể tích nổi, khối lượng thân...) để chúng luôn nhất quán với nhau.

Quy ước hệ trục:
  * Model / Gazebo: x tiến, y trái, z lên (FLU). Thế giới: ENU, z = 0 là mặt nước.
  * ArduPilot: thân FRD, thế giới NED. ArduPilotPlugin làm phép đổi trục bằng
    <modelXYZToAirplaneXForwardZDown> và <gazeboXYZToNED> (giống ví dụ BlueROV2
    chính thức của ardupilot_gazebo), cảm biến IMU được xoay 180° quanh x để
    đo trực tiếp theo FRD.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from typing import Any, Dict, List

import yaml

GRAVITY = 9.80665
NUM_THRUSTERS = 8   # frame Vectored-6DOF (FRAME_CONFIG = 2)
PWM_MIN = 1100
PWM_MAX = 1900


@dataclass
class Derived:
    """Các đại lượng tính ra từ YAML (in ra cho người dùng kiểm tra)."""

    total_mass: float
    base_mass: float
    displaced_volume: float
    hull_height: float
    weight_n: float
    buoyancy_n: float
    net_up_n: float
    com_z: float
    plugin_multiplier: float


def default_params_path() -> str:
    """Đường dẫn file YAML mặc định (trong share của package, hoặc trong src)."""
    try:
        from ament_index_python.packages import get_package_share_directory

        return os.path.join(
            get_package_share_directory('cps5_description'), 'config', 'cps5_vehicle.yaml'
        )
    except Exception:  # chạy ngoài ROS (scripts/generate_sdf.sh)
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(here, '..', 'config', 'cps5_vehicle.yaml')


def load_params(path: str | None = None) -> Dict[str, Any]:
    with open(path or default_params_path(), 'r', encoding='utf-8') as f:
        params = yaml.safe_load(f)
    validate(params)
    return params


def validate(p: Dict[str, Any]) -> None:
    """Chặn các giá trị vô lý trước khi chúng thành lỗi khó hiểu trong Gazebo."""
    v, th = p['vehicle'], p['thrusters']
    errors: List[str] = []
    if v['mass'] <= 0:
        errors.append('vehicle.mass phải > 0')
    if not -0.2 <= v['buoyancy_margin'] <= 0.3:
        errors.append('vehicle.buoyancy_margin nên nằm trong [-0.2, 0.3]')
    if p['water']['density'] < 900 or p['water']['density'] > 1100:
        errors.append('water.density phải khoảng 997 (nước ngọt) – 1025 (nước biển)')
    if len(th['list']) != NUM_THRUSTERS:
        errors.append(f'Vectored-6DOF cần đúng {NUM_THRUSTERS} thruster (thrusters.list)')
    if th['max_thrust'] <= 0:
        errors.append('thrusters.max_thrust phải > 0')
    if v['mass'] - th['prop_mass'] * len(th['list']) <= 0:
        errors.append('prop_mass quá lớn so với mass')
    for t in th['list']:
        if abs(_norm(t['axis']) - 1.0) > 1e-6:
            errors.append(f"{t['name']}.axis phải là vector đơn vị")
    for key in ('ixx', 'iyy', 'izz'):
        if v['inertia'][key] <= 0:
            errors.append(f'vehicle.inertia.{key} phải > 0')
    if errors:
        raise ValueError('Lỗi tham số CPS 5:\n  - ' + '\n  - '.join(errors))


def derive(p: Dict[str, Any]) -> Derived:
    v, th, hull = p['vehicle'], p['thrusters'], p['hull']
    rho = p['water']['density']
    total = float(v['mass'])
    base = total - th['prop_mass'] * len(th['list'])
    volume = total * (1.0 + v['buoyancy_margin']) / rho
    height = volume / (hull['length'] * hull['width'])
    weight = total * GRAVITY
    buoy = volume * rho * GRAVITY
    return Derived(
        total_mass=total,
        base_mass=base,
        displaced_volume=volume,
        hull_height=height,
        weight_n=weight,
        buoyancy_n=buoy,
        net_up_n=buoy - weight,
        com_z=-float(v['cob_above_com']),
        # ArduPilotPlugin: lệnh = (pwm_chuẩn_hóa + offset) * multiplier,
        # pwm_chuẩn_hóa ∈ [0, 1], offset = -0.5  =>  lực ∈ [-max, +max] N.
        plugin_multiplier=2.0 * th['max_thrust'],
    )


def summary(p: Dict[str, Any]) -> str:
    d = derive(p)
    lines = [
        'CPS 5 — các đại lượng suy ra từ YAML',
        f'  Tổng khối lượng          : {d.total_mass:.3f} kg (thân {d.base_mass:.3f} kg'
        f' + {len(p["thrusters"]["list"])} cánh)',
        f'  Thể tích chiếm chỗ       : {d.displaced_volume * 1000:.3f} L',
        f'  Hộp nổi (L x W x H)      : {p["hull"]["length"]:.3f} x {p["hull"]["width"]:.3f}'
        f' x {d.hull_height:.4f} m',
        f'  Trọng lượng / Lực nổi    : {d.weight_n:.2f} N / {d.buoyancy_n:.2f} N',
        f'  Lực nổi dư (dương = nổi) : {d.net_up_n:+.3f} N',
        f'  Trọng tâm so với tâm nổi : z = {d.com_z:+.3f} m',
        f'  Lực đẩy mỗi thruster     : ±{p["thrusters"]["max_thrust"]:.2f} N'
        f' (multiplier plugin = {d.plugin_multiplier:.2f})',
    ]
    if d.hull_height > p['hull']['width'] * 2:
        lines.append('  CẢNH BÁO: hộp nổi cao bất thường — kiểm tra lại hull.length/width.')
    return '\n'.join(lines)


# ----------------------------------------------------------------------------- helpers

def _norm(vec) -> float:
    return math.sqrt(sum(float(c) ** 2 for c in vec))


def _f(x: float) -> str:
    """Số gọn cho SDF."""
    return f'{x:.6g}'


def _vec(v) -> str:
    return ' '.join(_f(float(c)) for c in v)


def _axis_rpy(axis) -> str:
    """Hướng một hình trụ (mặc định dọc z) theo trục thruster — chỉ để hiển thị.

    Xoay pitch quanh y rồi yaw quanh z: trục z -> (sinP·cosY, sinP·sinY, cosP).
    Dùng được cho cả thruster xéo 45° của Vectored-6DOF.
    """
    ax, ay, az = (float(c) for c in axis)
    pitch = math.atan2(math.hypot(ax, ay), az)
    yaw = math.atan2(ay, ax) if math.hypot(ax, ay) > 1e-9 else 0.0
    return f'0 {_f(pitch)} {_f(yaw)}'


def _material(r: float, g: float, b: float) -> str:
    return (
        f'<material><ambient>{_f(r)} {_f(g)} {_f(b)} 1</ambient>'
        f'<diffuse>{_f(r)} {_f(g)} {_f(b)} 1</diffuse>'
        f'<specular>0.2 0.2 0.2 1</specular></material>'
    )


# ----------------------------------------------------------------------------- SDF

def model_sdf(p: Dict[str, Any]) -> str:
    """Trả về phần tử <model> (không kèm thẻ <sdf>) để nhúng vào world."""
    d = derive(p)
    v, th, hull = p['vehicle'], p['thrusters'], p['hull']
    name = v['name']
    rho = p['water']['density']
    sp = v['spawn']
    am = p['added_mass']
    dl, dq = p['drag']['linear'], p['drag']['quadratic']
    inertia = v['inertia']
    cam = p['camera']

    added_mass_xml = ''
    if am.get('enabled', True):
        added_mass_xml = (
            '<fluid_added_mass>'
            f'<xx>{_f(am["xx"])}</xx><yy>{_f(am["yy"])}</yy><zz>{_f(am["zz"])}</zz>'
            f'<pp>{_f(am["pp"])}</pp><qq>{_f(am["qq"])}</qq><rr>{_f(am["rr"])}</rr>'
            '</fluid_added_mass>'
        )

    # --- thân -------------------------------------------------------------------
    vis = p['visual']
    tube_r, tube_l = vis['tube_radius'], vis['tube_length']
    base_visuals = f'''
      <visual name="hull_tube">
        <pose>0 0 0 0 {_f(math.pi / 2)} 0</pose>
        <geometry><cylinder><radius>{_f(tube_r)}</radius><length>{_f(tube_l)}</length></cylinder></geometry>
        <material><ambient>0.7 0.85 0.95 1</ambient><diffuse>0.7 0.85 0.95 0.6</diffuse></material>
        <transparency>0.4</transparency>
      </visual>
      <visual name="front_dome">
        <pose>{_f(tube_l / 2)} 0 0 0 0 0</pose>
        <geometry><sphere><radius>{_f(tube_r)}</radius></sphere></geometry>
        <material><ambient>0.8 0.9 1 1</ambient><diffuse>0.8 0.9 1 0.5</diffuse></material>
        <transparency>0.6</transparency>
      </visual>
      <visual name="rear_endcap">
        <pose>{_f(-tube_l / 2 - 0.01)} 0 0 0 {_f(math.pi / 2)} 0</pose>
        <geometry><cylinder><radius>{_f(tube_r + 0.005)}</radius><length>0.02</length></cylinder></geometry>
        {_material(0.15, 0.15, 0.15)}
      </visual>
      <visual name="frame_left">
        <pose>0 {_f(hull['width'] / 2)} 0 0 0 0</pose>
        <geometry><box><size>{_f(hull['length'])} 0.012 0.05</size></box></geometry>
        {_material(0.95, 0.45, 0.1)}
      </visual>
      <visual name="frame_right">
        <pose>0 {_f(-hull['width'] / 2)} 0 0 0 0</pose>
        <geometry><box><size>{_f(hull['length'])} 0.012 0.05</size></box></geometry>
        {_material(0.95, 0.45, 0.1)}
      </visual>'''

    thruster_shrouds = ''.join(
        f'''
      <visual name="{t['name']}_shroud">
        <pose>{_vec(t['xyz'])} {_axis_rpy(t['axis'])}</pose>
        <geometry><cylinder><radius>{_f(th['propeller_diameter'] / 2 + 0.008)}</radius><length>0.06</length></cylinder></geometry>
        {_material(0.1, 0.1, 0.1)}
      </visual>'''
        for t in th['list']
    )

    camera_xml = ''
    if cam.get('enabled', True):
        camera_xml = f'''
      <sensor name="front_camera" type="camera">
        <pose>{_vec(cam['xyz'])} 0 0 0</pose>
        <always_on>1</always_on>
        <update_rate>{_f(cam['rate'])}</update_rate>
        <visualize>true</visualize>
        <topic>/{name}/camera/image</topic>
        <camera>
          <camera_info_topic>/{name}/camera/camera_info</camera_info_topic>
          <horizontal_fov>{_f(cam['hfov'])}</horizontal_fov>
          <image><width>{int(cam['width'])}</width><height>{int(cam['height'])}</height><format>R8G8B8</format></image>
          <clip><near>0.05</near><far>40</far></clip>
        </camera>
      </sensor>'''

    base_link = f'''
    <link name="base_link">
      <inertial>
        <!-- Trọng tâm thấp hơn tâm nổi (gốc link) => mô-men tự cân bằng -->
        <pose>0 0 {_f(d.com_z)} 0 0 0</pose>
        <mass>{_f(d.base_mass)}</mass>
        <inertia>
          <ixx>{_f(inertia['ixx'])}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{_f(inertia['iyy'])}</iyy><iyz>0</iyz><izz>{_f(inertia['izz'])}</izz>
        </inertia>
        {added_mass_xml}
      </inertial>

      <!-- Thể tích collision x mật độ nước = lực nổi (plugin Buoyancy).
           Chiều cao được tính tự động: {d.displaced_volume * 1000:.3f} L -->
      <collision name="buoyancy_hull">
        <geometry><box><size>{_f(hull['length'])} {_f(hull['width'])} {_f(d.hull_height)}</size></box></geometry>
      </collision>
{base_visuals}{thruster_shrouds}

      <!-- IMU xoay 180° quanh x: đo theo hệ FRD mà ArduPilot cần -->
      <sensor name="imu_sensor" type="imu">
        <pose>0 0 0 {_f(math.pi)} 0 0</pose>
        <always_on>1</always_on>
        <update_rate>{_f(p['imu']['rate'])}</update_rate>
      </sensor>
{camera_xml}
    </link>'''

    # --- thruster: mỗi cái là một link cánh nhỏ + khớp quay; lực đặt dọc trục khớp
    prop_inertia = 1e-6
    thruster_links, thruster_joints, thruster_plugins, controls = [], [], [], []
    for i, t in enumerate(th['list']):
        jn = f"{t['name']}_joint"
        thruster_links.append(f'''
    <link name="{t['name']}">
      <pose>{_vec(t['xyz'])} 0 0 0</pose>
      <inertial>
        <mass>{_f(th['prop_mass'])}</mass>
        <inertia><ixx>{prop_inertia}</ixx><ixy>0</ixy><ixz>0</ixz><iyy>{prop_inertia}</iyy><iyz>0</iyz><izz>{prop_inertia}</izz></inertia>
      </inertial>
      <visual name="prop">
        <pose>0 0 0 {_axis_rpy(t['axis'])}</pose>
        <geometry><cylinder><radius>{_f(th['propeller_diameter'] / 2)}</radius><length>0.004</length></cylinder></geometry>
        {_material(0.9, 0.9, 0.2)}
      </visual>
    </link>''')
        thruster_joints.append(f'''
    <joint name="{jn}" type="revolute">
      <parent>base_link</parent>
      <child>{t['name']}</child>
      <!-- Lực đẩy dương (PWM > 1500) hướng theo trục này -->
      <axis><xyz>{_vec(t['axis'])}</xyz></axis>
    </joint>''')
        coeff = th['thrust_coefficient'] * (1 if t.get('spin', 1) >= 0 else -1)
        thruster_plugins.append(f'''
    <plugin filename="gz-sim-thruster-system" name="gz::sim::systems::Thruster">
      <namespace>{name}</namespace>
      <joint_name>{jn}</joint_name>
      <thrust_coefficient>{_f(coeff)}</thrust_coefficient>
      <fluid_density>{_f(rho)}</fluid_density>
      <propeller_diameter>{_f(th['propeller_diameter'])}</propeller_diameter>
      <velocity_control>true</velocity_control>
      <use_angvel_cmd>false</use_angvel_cmd>
      <max_thrust_cmd>{_f(th['max_thrust'])}</max_thrust_cmd>
      <min_thrust_cmd>{_f(-th['max_thrust'])}</min_thrust_cmd>
    </plugin>''')
        controls.append(f'''
      <!-- SERVO{i + 1} (Motor {i + 1}) -> {t['name']} -->
      <control channel="{i}">
        <jointName>{jn}</jointName>
        <servo_min>{PWM_MIN}</servo_min>
        <servo_max>{PWM_MAX}</servo_max>
        <type>COMMAND</type>
        <cmd_topic>/model/{name}/joint/{jn}/cmd_thrust</cmd_topic>
        <offset>-0.5</offset>
        <multiplier>{_f(d.plugin_multiplier)}</multiplier>
      </control>''')

    hydro = f'''
    <plugin filename="gz-sim-hydrodynamics-system" name="gz::sim::systems::Hydrodynamics">
      <link_name>base_link</link_name>
      <water_density>{_f(rho)}</water_density>
      <!-- Added mass nằm trong <fluid_added_mass> của base_link, không khai báo ở đây -->
      <xU>{_f(dl['x'])}</xU><yV>{_f(dl['y'])}</yV><zW>{_f(dl['z'])}</zW>
      <kP>{_f(dl['k'])}</kP><mQ>{_f(dl['m'])}</mQ><nR>{_f(dl['n'])}</nR>
      <xUabsU>{_f(dq['x'])}</xUabsU><yVabsV>{_f(dq['y'])}</yVabsV><zWabsW>{_f(dq['z'])}</zWabsW>
      <kPabsP>{_f(dq['k'])}</kPabsP><mQabsQ>{_f(dq['m'])}</mQabsQ><nRabsR>{_f(dq['n'])}</nRabsR>
    </plugin>'''

    ardupilot = f'''
    <plugin name="ArduPilotPlugin" filename="ArduPilotPlugin">
      <fdm_addr>127.0.0.1</fdm_addr>
      <fdm_port_in>9002</fdm_port_in>
      <connectionTimeoutMaxCount>5</connectionTimeoutMaxCount>
      <lock_step>1</lock_step>
      <!-- Thân: FLU (Gazebo) -> FRD (ArduPilot) -->
      <modelXYZToAirplaneXForwardZDown degrees="true">0 0 0 180 0 0</modelXYZToAirplaneXForwardZDown>
      <!-- Thế giới: ENU (Gazebo) -> NED (ArduPilot) -->
      <gazeboXYZToNED degrees="true">0 0 0 180 0 90</gazeboXYZToNED>
      <imuName>imu_sensor</imuName>
{''.join(controls)}
    </plugin>'''

    odom = f'''
    <!-- Vị trí "thật" (ground truth) để script kiểm tra so sánh -->
    <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
      <odom_frame>world</odom_frame>
      <robot_base_frame>{name}/base_link</robot_base_frame>
      <dimensions>3</dimensions>
      <odom_publish_frequency>50</odom_publish_frequency>
      <odom_topic>/model/{name}/odometry</odom_topic>
    </plugin>'''

    return f'''
  <model name="{name}">
    <pose>{_f(sp['x'])} {_f(sp['y'])} {_f(sp['z'])} 0 0 {_f(sp['yaw'])}</pose>
{base_link}
{''.join(thruster_links)}
{''.join(thruster_joints)}
{''.join(thruster_plugins)}
{hydro}
{ardupilot}
{odom}
  </model>'''


def standalone_model_sdf(p: Dict[str, Any]) -> str:
    """SDF chỉ có model (để mở xem riêng / debug)."""
    return f'<?xml version="1.0"?>\n<sdf version="1.11">{model_sdf(p)}\n</sdf>\n'


def main() -> None:
    import argparse

    ap = argparse.ArgumentParser(description='In các đại lượng suy ra và (tuỳ chọn) ghi model SDF.')
    ap.add_argument('--params', default=None, help='file YAML (mặc định: cps5_vehicle.yaml)')
    ap.add_argument('--out', default=None, help='ghi model.sdf vào đường dẫn này')
    args = ap.parse_args()
    params = load_params(args.params)
    print(summary(params))
    if args.out:
        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
        with open(args.out, 'w', encoding='utf-8') as f:
            f.write(standalone_model_sdf(params))
        print(f'Đã ghi {args.out}')


if __name__ == '__main__':
    main()
