"""Sinh file world SDF (bể nước + robot CPS 5) cho Gazebo Harmonic.

Robot được nhúng thẳng vào world (không spawn riêng) nên chỉ cần một lệnh
`gz sim <world>` là có đủ. File được sinh lại mỗi lần chạy simulation, vì vậy
sửa YAML là đủ — không bao giờ sửa file .sdf sinh ra.
"""

from __future__ import annotations

import os
from typing import Any, Dict

import yaml

from cps5_description import model as cps5_model


def default_world_params_path() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory

        return os.path.join(get_package_share_directory('cps5_gazebo'), 'config', 'pool_world.yaml')
    except Exception:
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(here, '..', 'config', 'pool_world.yaml')


def load_world_params(path: str | None = None) -> Dict[str, Any]:
    with open(path or default_world_params_path(), 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def _f(x: float) -> str:
    return f'{x:.6g}'


def _color(c) -> str:
    r, g, b = (float(x) for x in c)
    return (f'<material><ambient>{_f(r)} {_f(g)} {_f(b)} 1</ambient>'
            f'<diffuse>{_f(r)} {_f(g)} {_f(b)} 1</diffuse></material>')


def _static_box(name: str, size, xyz, color, collision: bool = True, transparency: float = 0.0) -> str:
    sx, sy, sz = (_f(float(s)) for s in size)
    geom = f'<geometry><box><size>{sx} {sy} {sz}</size></box></geometry>'
    col = f'<collision name="c">{geom}</collision>' if collision else ''
    tr = f'<transparency>{_f(transparency)}</transparency>' if transparency else ''
    return f'''
    <model name="{name}"><static>true</static>
      <pose>{_f(xyz[0])} {_f(xyz[1])} {_f(xyz[2])} 0 0 0</pose>
      <link name="link">{col}<visual name="v">{geom}{_color(color)}{tr}</visual></link>
    </model>'''


def _object(name: str, o: Dict[str, Any]) -> str:
    t = o['type']
    if t == 'sphere':
        geom = f'<sphere><radius>{_f(o["radius"])}</radius></sphere>'
    elif t == 'box':
        geom = '<box><size>' + ' '.join(_f(s) for s in o['size']) + '</size></box>'
    elif t == 'cylinder':
        geom = f'<cylinder><radius>{_f(o["radius"])}</radius><length>{_f(o["length"])}</length></cylinder>'
    else:
        raise ValueError(f'Kiểu vật thể không hỗ trợ: {t}')
    x, y, z = o['xyz']
    return f'''
    <model name="{name}"><static>true</static>
      <pose>{_f(x)} {_f(y)} {_f(z)} 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry>{geom}</geometry></collision>
        <visual name="v"><geometry>{geom}</geometry>{_color(o['color'])}</visual>
      </link>
    </model>'''


def world_sdf(vehicle: Dict[str, Any], world: Dict[str, Any], render_engine: str = 'ogre2') -> str:
    w, pool = world['world'], world['pool']
    rho = vehicle['water']['density']
    name = vehicle['vehicle']['name']
    L, W, D, t = pool['length'], pool['width'], pool['depth'], pool['wall_thickness']
    wall_h = D + 0.5
    wall_z = -D + wall_h / 2
    grey = [0.75, 0.8, 0.82]

    pool_xml = ''.join([
        _static_box('pool_floor', [L, W, t], [0, 0, -D - t / 2], [0.6, 0.7, 0.75]),
        _static_box('wall_px', [t, W + 2 * t, wall_h], [L / 2 + t / 2, 0, wall_z], grey),
        _static_box('wall_nx', [t, W + 2 * t, wall_h], [-L / 2 - t / 2, 0, wall_z], grey),
        _static_box('wall_py', [L, t, wall_h], [0, W / 2 + t / 2, wall_z], grey),
        _static_box('wall_ny', [L, t, wall_h], [0, -W / 2 - t / 2, wall_z], grey),
        # Khối nước chỉ để nhìn (không va chạm). Lực nổi do plugin Buoyancy tính.
        _static_box('water_volume', [L, W, D], [0, 0, -D / 2], [0.1, 0.4, 0.6],
                    collision=False, transparency=0.85),
    ])
    objects_xml = ''.join(_object(k, o) for k, o in (world.get('objects') or {}).items())

    sensors_plugin = ''
    if vehicle.get('camera', {}).get('enabled', True):
        sensors_plugin = f'''
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>{render_engine}</render_engine>
    </plugin>'''

    return f'''<?xml version="1.0"?>
<!-- FILE SINH TỰ ĐỘNG — đừng sửa. Sửa cps5_vehicle.yaml / pool_world.yaml rồi chạy lại. -->
<sdf version="1.11">
  <world name="{w['name']}">
    <physics name="1ms" type="ignored">
      <max_step_size>{_f(w['step_size'])}</max_step_size>
      <real_time_factor>{_f(w['real_time_factor'])}</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>{sensors_plugin}

    <!-- Lực nổi: mật độ {_f(rho)} kg/m³ dưới z = 0, không khí (1 kg/m³) phía trên -->
    <plugin filename="gz-sim-buoyancy-system" name="gz::sim::systems::Buoyancy">
      <graded_buoyancy>
        <default_density>{_f(rho)}</default_density>
        <density_change>
          <above_depth>0</above_depth>
          <density>1</density>
        </density_change>
      </graded_buoyancy>
      <enable>{name}</enable>
    </plugin>

    <scene>
      <ambient>0.5 0.6 0.7 1</ambient>
      <background>0.25 0.5 0.65 1</background>
      <shadows>false</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.2 -0.9</direction>
    </light>
{pool_xml}
{objects_xml}
{cps5_model.model_sdf(vehicle)}
  </world>
</sdf>
'''


def write_world(out_dir: str,
                vehicle_params_path: str | None = None,
                world_params_path: str | None = None,
                render_engine: str = 'ogre2',
                camera: bool | None = None) -> str:
    """Sinh world + model, ghi ra out_dir, trả về đường dẫn file world."""
    vehicle = cps5_model.load_params(vehicle_params_path)
    if camera is not None:
        vehicle.setdefault('camera', {})['enabled'] = bool(camera)
    world = load_world_params(world_params_path)
    os.makedirs(out_dir, exist_ok=True)
    world_path = os.path.join(out_dir, f"{world['world']['name']}.sdf")
    with open(world_path, 'w', encoding='utf-8') as f:
        f.write(world_sdf(vehicle, world, render_engine))
    with open(os.path.join(out_dir, f"{vehicle['vehicle']['name']}_model.sdf"), 'w', encoding='utf-8') as f:
        f.write(cps5_model.standalone_model_sdf(vehicle))
    return world_path


def main() -> None:
    import argparse

    ap = argparse.ArgumentParser(description='Sinh world SDF cho simulation CPS 5.')
    ap.add_argument('--out', default=os.path.expanduser('~/.cps5/generated'))
    ap.add_argument('--vehicle', default=None, help='cps5_vehicle.yaml')
    ap.add_argument('--world', default=None, help='pool_world.yaml')
    ap.add_argument('--render-engine', default='ogre2', choices=['ogre2', 'ogre'])
    args = ap.parse_args()
    path = write_world(args.out, args.vehicle, args.world, args.render_engine)
    print(cps5_model.summary(cps5_model.load_params(args.vehicle)))
    print(f'World: {path}')


if __name__ == '__main__':
    main()
