"""The graded straight: its length follows from the speed profile it must hold.

    L = sum (v_i^2 - v_{i-1}^2) / (2a) + sum v_i T + v_top^2 / (2d) + run-out

With a = 2.5, d = 2.5, T = 3 s and 0 -> 2 -> 6 -> 11 m/s: 115.4 m, 17.8 s
(docs/design/rc-sim-design.md, "The track"). The written map is one straight
lane, split into a lanelet per segment so each carries its speed limit, plus
a 1 m lead-in behind the start pose, a stop line, a flat point cloud for the
map loader, and track.yaml with the profile and the start and goal poses.

    rc_sim_make_track --speeds 2,6,11 --hold 3 --accel 2.5 --decel 2.5 [--out DIR]

Refused: speeds that do not rise, a speed the model cannot hold with 5 %
headroom, an acceleration beyond the rear grip or the motor. Warned: a
deceleration the rear brake cannot reach (stop_overshoot says how far past
the stop line that puts the car; the grader's stop error does not allow for
it).
"""
import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import sys
from typing import List, Tuple
import xml.etree.ElementTree as ET

import yaml

from autosdv_rc_sim.detmath import sqrt
from autosdv_rc_sim.drivetrain import brake_decel, drive_force, grip_accel, road_load, top_speed
from autosdv_rc_sim.params import load_vehicle, VehicleParams

LANE_WIDTH = 1.0
LEAD_IN = 1.0
DEFAULT_TRACK_NAME = 'straight_2_6_11'


class ProfileError(ValueError):
    """A profile the model cannot drive."""


@dataclass(frozen=True)
class Segment:
    kind: str     # accel | hold | brake | runout
    s0: float
    s1: float
    v0: float
    v1: float

    @property
    def length(self) -> float:
        return self.s1 - self.s0

    @property
    def duration(self) -> float:
        if self.kind == 'hold':
            return self.length / self.v0
        if self.kind in ('accel', 'brake'):
            return 2.0 * self.length / (self.v0 + self.v1)
        return 0.0


@dataclass(frozen=True)
class TrackProfile:
    speeds: Tuple[float, ...]
    hold: float
    accel: float
    decel: float
    runout: float
    engage_velocity: float
    segments: Tuple[Segment, ...]

    @property
    def stop_line_s(self) -> float:
        return self.segments[-2].s1

    @property
    def length(self) -> float:
        return self.segments[-1].s1

    def v_target(self, s: float) -> float:
        """The speed Autoware's planner commands at position s along the straight."""
        if s >= self.stop_line_s:
            return 0.0
        for seg in self.segments:
            if s < seg.s1:
                if seg.kind == 'hold':
                    return seg.v0
                if seg.kind == 'accel':
                    v = sqrt(seg.v0 * seg.v0 + 2.0 * self.accel * max(0.0, s - seg.s0))
                    return max(min(v, seg.v1), self.engage_velocity)
                return min(seg.v0, sqrt(2.0 * self.decel * (self.stop_line_s - s)))
        return 0.0

    def table(self) -> str:
        rows = ['| Segment | Length | Time |', '|---|---|---|']
        for seg in self.segments:
            if seg.kind == 'accel':
                name = f'accelerate {seg.v0:g} → {seg.v1:g} m/s'
            elif seg.kind == 'hold':
                name = f'hold {seg.v0:g} m/s'
            elif seg.kind == 'brake':
                name = f'brake {seg.v0:g} m/s → stop on the stop line'
            else:
                name = 'run-out beyond the stop line'
            time = '—' if seg.kind == 'runout' else f'{seg.duration:.1f} s'
            rows.append(f'| {name} | {seg.length:.1f} m | {time} |')
        total_t = sum(seg.duration for seg in self.segments)
        rows.append(f'| **total** | **{self.length:.1f} m** | **{total_t:.1f} s** |')
        return '\n'.join(rows)


def make_profile(speeds, hold: float = 3.0, accel: float = 2.5, decel: float = 2.5,
                 runout: float = 10.0, engage_velocity: float = 0.25) -> TrackProfile:
    speeds = tuple(float(v) for v in speeds)
    if not speeds:
        raise ProfileError('need at least one speed')
    if not all(math.isfinite(x) for x in speeds + (hold, accel, decel, runout, engage_velocity)):
        raise ProfileError('speeds, hold, accel, decel, runout and engage_velocity must be finite')
    previous = 0.0
    for v in speeds:
        if v <= previous:
            raise ProfileError(f'speeds must rise strictly from 0: {speeds}')
        previous = v
    if hold <= 0.0 or accel <= 0.0 or decel <= 0.0 or runout < 0.0:
        raise ProfileError('hold, accel and decel must be positive and runout not negative')
    segments, s, v0 = [], 0.0, 0.0
    for v in speeds:
        length = (v * v - v0 * v0) / (2.0 * accel)
        segments.append(Segment('accel', s, s + length, v0, v))
        s += length
        segments.append(Segment('hold', s, s + v * hold, v, v))
        s += v * hold
        v0 = v
    length = v0 * v0 / (2.0 * decel)
    segments.append(Segment('brake', s, s + length, v0, 0.0))
    s += length
    segments.append(Segment('runout', s, s + runout, 0.0, 0.0))
    return TrackProfile(speeds, float(hold), float(accel), float(decel), float(runout),
                        float(engage_velocity), tuple(segments))


def stop_overshoot(profile: TrackProfile, vehicle: VehicleParams, level: float = 1.0) -> float:
    """Metres past the stop line that physics forces: braking from the top speed at the
    profile's decel where the car can, at ESC brake `level` where it cannot."""
    v_top = profile.speeds[-1]
    n = 2000
    dv = v_top / n
    distance = 0.0
    for i in range(n):
        v = (i + 0.5) * dv
        distance += v * dv / min(profile.decel, brake_decel(vehicle, v, level))
    return max(0.0, distance - profile.segments[-2].length)


def check_feasible(profile: TrackProfile, vehicle: VehicleParams) -> List[str]:
    """Raise ProfileError for a profile the model cannot drive; return warnings."""
    top = top_speed(vehicle)
    for v in profile.speeds:
        if v > 0.95 * top:
            raise ProfileError(
                f'{v:g} m/s cannot be held: the model tops out at {top:.2f} m/s and a hold '
                f'needs 5 % headroom')
        need = vehicle.effective_mass * profile.accel * 1.1 + road_load(vehicle, v)
        if drive_force(vehicle, 1.0, v) < need:
            raise ProfileError(f'the motor cannot accelerate at {profile.accel:g} m/s^2 up to {v:g} m/s')
    if profile.accel > 0.9 * grip_accel(vehicle):
        raise ProfileError(
            f'{profile.accel:g} m/s^2 is beyond the rear grip (launch limit '
            f'{grip_accel(vehicle):.2f} m/s^2)')
    warnings = []
    overshoot = stop_overshoot(profile, vehicle)
    if overshoot > 0.05:
        warnings.append(
            f'the rear brake gives {brake_decel(vehicle, 1.0):.2f}-'
            f'{brake_decel(vehicle, profile.speeds[-1]):.2f} m/s^2, not {profile.decel:g}: '
            f'expect {overshoot:.1f} m past the stop line')
    return warnings


def _osm(profile: TrackProfile) -> str:
    half = LANE_WIDTH / 2.0
    # The lead-in is a lanelet of its own (limit: the first speed), then one per segment
    stations = [-LEAD_IN, 0.0] + [seg.s1 for seg in profile.segments]
    top = profile.speeds[-1]
    limits = [profile.speeds[0]] + [
        seg.v1 if seg.kind == 'accel' else (seg.v0 if seg.kind == 'hold' else top)
        for seg in profile.segments]
    root = ET.Element('osm', {'version': '0.6', 'generator': 'autosdv_rc_sim make-track'})
    ET.SubElement(root, 'MetaInfo', {'format_version': '1', 'map_version': '1'})
    next_id = [1]

    def new_id():
        next_id[0] += 1
        return next_id[0] - 1

    def node(x, y):
        nid = new_id()
        el = ET.SubElement(root, 'node', {'id': str(nid), 'lat': '0', 'lon': '0'})
        for k, v in (('local_x', f'{x:.4f}'), ('local_y', f'{y:.4f}'), ('ele', '0.0000')):
            ET.SubElement(el, 'tag', {'k': k, 'v': v})
        return nid

    def way(nodes, tags):
        wid = new_id()
        el = ET.SubElement(root, 'way', {'id': str(wid)})
        for n in nodes:
            ET.SubElement(el, 'nd', {'ref': str(n)})
        for k, v in tags:
            ET.SubElement(el, 'tag', {'k': k, 'v': v})
        return wid

    left = [node(s, half) for s in stations]
    right = [node(s, -half) for s in stations]
    stop = [node(profile.stop_line_s, half), node(profile.stop_line_s, -half)]
    line = (('type', 'line_thin'), ('subtype', 'solid'))
    bounds = [(way([left[i], left[i + 1]], line), way([right[i], right[i + 1]], line))
              for i in range(len(stations) - 1)]
    way(stop, (('type', 'stop_line'),))
    for (lw, rw), limit in zip(bounds, limits, strict=True):
        rel = ET.SubElement(root, 'relation', {'id': str(new_id())})
        ET.SubElement(rel, 'member', {'type': 'way', 'role': 'left', 'ref': str(lw)})
        ET.SubElement(rel, 'member', {'type': 'way', 'role': 'right', 'ref': str(rw)})
        for k, v in (('type', 'lanelet'), ('subtype', 'road'), ('location', 'urban'),
                     ('one_way', 'yes'), ('participant:vehicle', 'yes'),
                     ('speed_limit', f'{limit * 3.6:.1f}')):
            ET.SubElement(rel, 'tag', {'k': k, 'v': v})
    ET.indent(root)
    return '<?xml version="1.0" encoding="UTF-8"?>\n' + ET.tostring(root, encoding='unicode') + '\n'


def _pcd(profile: TrackProfile) -> str:
    points = []
    n = int(round((profile.length + LEAD_IN) / 0.25))
    for i in range(n + 1):
        x = -LEAD_IN + 0.25 * i
        for y in (LANE_WIDTH / 2.0, -LANE_WIDTH / 2.0):
            points.append(f'{x:.3f} {y:.3f} 0.000')
    header = ['# .PCD v0.7 - Point Cloud Data file format', 'VERSION 0.7', 'FIELDS x y z',
              'SIZE 4 4 4', 'TYPE F F F', 'COUNT 1 1 1', f'WIDTH {len(points)}', 'HEIGHT 1',
              'VIEWPOINT 0 0 0 1 0 0 0', f'POINTS {len(points)}', 'DATA ascii']
    return '\n'.join(header + points) + '\n'


def _track_yaml(profile: TrackProfile) -> str:
    data = {
        'speeds': list(profile.speeds), 'hold': profile.hold, 'accel': profile.accel,
        'decel': profile.decel, 'runout': profile.runout,
        'engage_velocity': profile.engage_velocity, 'lead_in': LEAD_IN, 'lane_width': LANE_WIDTH,
        'start_pose': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
        'goal_pose': {'x': round(profile.stop_line_s, 4), 'y': 0.0, 'yaw': 0.0},
        'stop_line_s': round(profile.stop_line_s, 4), 'length': round(profile.length, 4),
    }
    return '# Generated by rc_sim_make_track; regenerate, do not edit.\n' + yaml.safe_dump(
        data, sort_keys=False, default_flow_style=None)


def write_track(profile: TrackProfile, out_dir) -> None:
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    (out / 'lanelet2_map.osm').write_text(_osm(profile))
    (out / 'pointcloud_map.pcd').write_text(_pcd(profile))
    (out / 'map_projector_info.yaml').write_text('projector_type: local\n')
    (out / 'track.yaml').write_text(_track_yaml(profile))


def _read(directory) -> dict:
    with open(Path(directory) / 'track.yaml') as f:
        return yaml.safe_load(f)


def load_track(directory) -> TrackProfile:
    d = _read(directory)
    return make_profile(d['speeds'], d['hold'], d['accel'], d['decel'], d['runout'],
                        d['engage_velocity'])


def track_poses(directory) -> dict:
    d = _read(directory)
    return {'start_pose': d['start_pose'], 'goal_pose': d['goal_pose']}


def default_track_dir() -> Path:
    source = Path(__file__).resolve().parents[1] / 'tracks' / DEFAULT_TRACK_NAME
    if source.is_dir():
        return source
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('autosdv_rc_sim')) / 'tracks' / DEFAULT_TRACK_NAME


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='Size and write the graded straight.')
    ap.add_argument('--speeds', default='2,6,11', help='speeds to hold [m/s], rising')
    ap.add_argument('--hold', type=float, default=3.0, help='hold time per speed [s]')
    ap.add_argument('--accel', type=float, default=2.5, help="the planner's acceleration [m/s^2]")
    ap.add_argument('--decel', type=float, default=2.5, help="the planner's deceleration [m/s^2]")
    ap.add_argument('--runout', type=float, default=10.0, help='beyond the stop line [m]')
    ap.add_argument('--variant', default='nominal', help='vehicle model variant to check against')
    ap.add_argument('--out', default='', help='output directory (default tmp/rc_sim/tracks/straight_<speeds>)')
    args = ap.parse_args(argv)
    try:
        speeds = [float(x) for x in args.speeds.split(',') if x.strip()]
        profile = make_profile(speeds, args.hold, args.accel, args.decel, args.runout)
        warnings = check_feasible(profile, load_vehicle(variant=args.variant))
    except (ProfileError, ValueError) as e:
        print(f'error: {e}', file=sys.stderr)
        return 2
    print(profile.table())
    for w in warnings:
        print(f'warning: {w}', file=sys.stderr)
    out = args.out or 'tmp/rc_sim/tracks/straight_' + '_'.join(f'{v:g}' for v in profile.speeds)
    write_track(profile, out)
    print(f'wrote {out}: lanelet2_map.osm, pointcloud_map.pcd, map_projector_info.yaml, track.yaml')
    print(f'drive it: just sim rc track:={out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
