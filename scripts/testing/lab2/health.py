#!/usr/bin/env python3
"""Lab 2 car-day health monitor: topic rates, CPU, memory, GPU, temperatures.

Run it beside `just lab2 run` (or `loc-check`, `vehicle`, `bench`) in a second
tmux window; stop it with Ctrl-C, or give it a duration:

    just lab2 health                      until Ctrl-C
    just lab2 health --duration 120       two minutes
    just lab2 health --with-run           file the result with the run's bag
    just lab2 health --kernel             also one kernel_cpu_report.sh snapshot at the end

It starts, together:

  topics       receive times of the topics that matter, from the graph (it
               subscribes as soon as a publisher appears, so it may start
               before the stack)
  cpu/memory   /proc: per-core busy, load average, memory and swap, and the
               ROS processes' CPU (the top consumers by node name)
  gpu          scripts/profiling/jetson_gpu_sampler.py (Jetson: load, clock,
               rails, temperatures) -> gpu.csv; nvidia-smi on a workstation
  tegrastats   if installed -> tegrastats.log, summarised with
               scripts/testing/localization/tegrastats_summary.py
  thermal      /sys/class/thermal zones, against their passive trip points,
               and the INA3221 over-current event counters (Jetson)

and on stop writes summary.txt and summary.json to rosbags/health/<date>_<time>/
(with --with-run: <bag_dir>/run_logs/<bag>/health_<time>/, the newest lab2_* bag
started after this monitor, the same place `just lab2 run` puts its logs).

Thresholds are the stack's own, not new ones: the rates are
waypoint_preflight.RATES (fail/warn Hz), gaps are topic_state_monitor's timeout
(config/system/component_state_monitor/topics-waypoint.yaml), the marker fixes
are preflight's 2 fixes/s and localization_monitor's error_age.

The summarising functions below take plain data and import nothing from ROS:
test_health.py tests them without a stack.
"""

import argparse
import csv
import glob
import json
import os
import platform
import re
import shutil
import signal
import statistics
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
GPU_SAMPLER = REPO / 'scripts/profiling/jetson_gpu_sampler.py'
KERNEL_REPORT = REPO / 'scripts/profiling/kernel_cpu_report.sh'
TEGRASTATS_SUMMARY = REPO / 'scripts/testing/localization/tegrastats_summary.py'
MONITOR_TOPICS = REPO / 'src/launcher/autosdv_launch/config/system/component_state_monitor/topics-waypoint.yaml'
LOC_MONITOR = REPO / 'src/lab/autosdv_waypoint_lab/config/localization_monitor.param.yaml'
JETSON_GPU_LOAD = '/sys/devices/platform/gpu.0/load'

OK, WARN, FAIL = 'OK', 'WARN', 'FAIL'
ORDER = {OK: 0, WARN: 1, FAIL: 2}

CAMERA_INFO = '/sensing/camera/zedxm/rgb/color/rect/camera_info'
FIX = '/localization/pose_estimator/pose_with_covariance'
CONTROL_CMD = '/control/command/control_cmd'
PWM = '/autosdv/actuator_node/debug/pwm_values'
ALL_MODES = ('vehicle', 'bench', 'mock')

# used only if the stack's own files cannot be read (a partial checkout)
DEFAULT_MONITOR = {'warn_rate': 5.0, 'error_rate': 1.0, 'timeout': 1.0}
DEFAULT_ERROR_AGE = 5.0
FIX_WARN_RATE = 2.0          # waypoint_preflight: PASS if >= 2 accepted fixes/s, else WARN

# a busy core this far below its frequency cap counts as throttled
FREQ_THROTTLE_RATIO = 0.9
BUSY_CORE_PCT = 90.0
MEM_WARN_PCT = 90.0
CORE_MEDIAN_WARN_PCT = 90.0
# swap already in use is someone else's history; swap that grows during the run is ours
SWAP_GROWTH_WARN_MB = 64.0


# ----------------------------------------------------------------- statistics

def percentile(values, q):
    """Linear-interpolated percentile, q in [0, 100]; None for no values."""
    vals = sorted(values)
    if not vals:
        return None
    if len(vals) == 1:
        return float(vals[0])
    k = (len(vals) - 1) * q / 100.0
    lo = int(k)
    hi = min(lo + 1, len(vals) - 1)
    return float(vals[lo] + (vals[hi] - vals[lo]) * (k - lo))


def median(values):
    vals = list(values)
    return float(statistics.median(vals)) if vals else None


def rate_stats(times, t_start, t_end, bin_s=1.0):
    """Rate and staleness of one topic from its receive times (seconds, any clock).

    The window is [t_start, t_end): from when the subscription was made to the
    stop. Rates are counted in whole bins of bin_s (a stream that stops gives 0 Hz
    bins, not a missing sample); max_gap is the longest silence between messages
    or from the last message to the stop, so a stream that died counts. The head
    (subscription to first message) is discovery, not a gap, and is excluded.
    """
    times = sorted(t for t in times if t_start <= t < t_end)
    span = t_end - t_start
    out = {'n': len(times), 'window_s': round(span, 2), 'mean_hz': None, 'median_hz': None,
           'p5_hz': None, 'min_hz': None, 'max_gap_s': None, 'age_at_end_s': None}
    if span <= 0:
        return out
    if times:
        first = times[0]
        active = t_end - first
        out['mean_hz'] = round((len(times) - 1) / active, 2) if active > 0 and len(times) > 1 else 0.0
        gaps = [b - a for a, b in zip(times, times[1:])] + [t_end - times[-1]]
        out['max_gap_s'] = round(max(gaps), 3)
        out['age_at_end_s'] = round(t_end - times[-1], 3)
        nbins = int((t_end - first) // bin_s)
        if nbins >= 1:
            counts = [0] * nbins
            for t in times:
                i = int((t - first) // bin_s)
                if i < nbins:
                    counts[i] += 1
            rates = [c / bin_s for c in counts]
            out['median_hz'] = round(median(rates), 2)
            out['p5_hz'] = round(percentile(rates, 5), 2)
            out['min_hz'] = round(min(rates), 2)
    return out


def stack_stopped_at(times_by_topic, t_end, quiet_s=1.0):
    """When every topic went silent together before the stop: the last message of any
    topic, if that is more than quiet_s before t_end, else None.

    The stack shutting down (a finished `just lab2 run`) is not a gap in each topic:
    the caller ends the window there and says so once."""
    last = [max(t) for t in times_by_topic.values() if t]
    if not last:
        return None
    stopped = max(last)
    return stopped if t_end - stopped > quiet_s else None


def topic_verdict(spec, stats, mode):
    """(level, detail) for one topic, or None if the topic is not expected in mode."""
    label = spec['label']
    if mode not in spec.get('modes', ALL_MODES):
        return None
    if stats is None or stats['n'] == 0:
        why = 'no publisher' if stats is None else 'publisher, but no message'
        return FAIL, f'{label}: {why} in {spec["topic"]}'
    med = stats['median_hz']
    if med is None:
        return WARN, f'{label}: {stats["n"]} messages, too short a window for a rate'
    level = OK
    notes = []
    if spec['kind'] == 'fix':
        if med < spec['warn_hz']:
            level = WARN
            notes.append(f'median under {spec["warn_hz"]:g}/s (no marker in view for stretches?)')
        gap = stats['max_gap_s']
        if gap > spec['fail_gap_s']:
            level = FAIL
            notes.append(f'gap {gap:.1f} s over error_age {spec["fail_gap_s"]:g} s: an MRM if engaged')
        elif gap > spec['warn_gap_s']:
            level = max(level, WARN, key=ORDER.get)
            notes.append(f'gap {gap:.1f} s over half error_age')
    else:
        if med < spec['fail_hz']:
            level = FAIL
            notes.append(f'median under {spec["fail_hz"]:g} Hz')
        elif med < spec['warn_hz']:
            level = WARN
            notes.append(f'median under {spec["warn_hz"]:g} Hz')
        if stats['p5_hz'] < spec['fail_hz'] and level == OK:
            level = WARN
            notes.append(f'p5 under {spec["fail_hz"]:g} Hz: dips')
        if stats['max_gap_s'] > spec['timeout_s']:
            level = FAIL
            notes.append(f'gap {stats["max_gap_s"]:.2f} s over the {spec["timeout_s"]:g} s monitor '
                         'timeout: an MRM if engaged')
    detail = (f'{label}: median {med:.1f} Hz, p5 {stats["p5_hz"]:.1f}, min {stats["min_hz"]:.1f}, '
              f'max gap {stats["max_gap_s"]:.2f} s')
    if notes:
        detail += ' -- ' + '; '.join(notes)
    return level, detail


def cpu_busy(prev, cur):
    """Per-core busy % between two /proc/stat readings ({core: (busy, total)} jiffies)."""
    out = {}
    for core, (b1, t1) in cur.items():
        if core in prev:
            b0, t0 = prev[core]
            dt = t1 - t0
            out[core] = 100.0 * (b1 - b0) / dt if dt > 0 else 0.0
    return out


def parse_proc_stat(text):
    """{'cpu': (busy, total), 'cpu0': ...} jiffies; busy excludes idle and iowait."""
    out = {}
    for line in text.splitlines():
        if not line.startswith('cpu'):
            continue
        parts = line.split()
        vals = [int(v) for v in parts[1:]]
        idle = vals[3] + (vals[4] if len(vals) > 4 else 0)
        total = sum(vals[:8])        # guest time is already counted in user
        out[parts[0]] = (total - idle, total)
    return out


def parse_meminfo(text):
    """MemTotal, MemAvailable, SwapTotal, SwapFree in MB."""
    vals = {}
    for line in text.splitlines():
        key, _, rest = line.partition(':')
        if key in ('MemTotal', 'MemAvailable', 'SwapTotal', 'SwapFree'):
            vals[key] = int(rest.split()[0]) / 1024.0
    return vals


def cpu_summary(samples):
    """samples: list of {core: busy %} with 'cpu' the total. Peak/median per core."""
    cores = sorted({c for s in samples for c in s if c != 'cpu'},
                   key=lambda c: int(c[3:]) if c[3:].isdigit() else 0)
    per = []
    for c in cores:
        vals = [s[c] for s in samples if c in s]
        per.append({'core': c, 'median_pct': round(median(vals), 1), 'peak_pct': round(max(vals), 1)})
    tot = [s['cpu'] for s in samples if 'cpu' in s]
    return {'cores': per,
            'total_median_pct': round(median(tot), 1) if tot else None,
            'total_peak_pct': round(max(tot), 1) if tot else None,
            'samples': len(samples)}


ROS_HINTS = ('--ros-args', '/opt/ros/', '/opt/autoware/', 'play_launch', 'component_container',
             '/install/')


def process_name(comm, cmdline):
    """A ROS process's node name (__node:=X, or the container's), else its command."""
    m = re.search(r'__node:=([^\s]+)', cmdline)
    if m:
        ns = re.search(r'__ns:=([^\s]+)', cmdline)
        name = m.group(1)
        return f'{ns.group(1).rstrip("/")}/{name}' if ns and ns.group(1) != '/' else name
    # comm is cut at 15 characters: name the executable (or the script) from argv
    argv = cmdline.split()
    if not argv:
        return comm
    exe = os.path.basename(argv[0])
    if exe.startswith('python') and len(argv) > 1 and not argv[1].startswith('-'):
        return os.path.basename(argv[1])
    if exe == 'ros2':
        return ' '.join(['ros2'] + argv[1:2])
    return exe or comm


def is_ros_process(cmdline):
    return any(h in cmdline for h in ROS_HINTS)


class ProcTracker:
    """CPU seconds, peak CPU% and peak RSS per process name, from successive snapshots.

    A snapshot is {pid: (name, cpu_seconds, rss_mb)}; update(snapshot, dt) adds the
    interval since the previous one. A pid seen first in a snapshot contributes
    from the next interval on (its earlier CPU is not in this window).
    """

    def __init__(self):
        self.prev = {}
        self.by_name = {}
        self.elapsed = 0.0

    def update(self, snap, dt):
        if self.prev and dt > 0:
            self.elapsed += dt
            interval = {}
            for pid, (name, cpu_s, rss) in snap.items():
                if pid in self.prev and self.prev[pid][0] == name:
                    d = max(0.0, cpu_s - self.prev[pid][1])
                    interval[name] = interval.get(name, 0.0) + d
            for name, d in interval.items():
                e = self.by_name.setdefault(name, {'cpu_s': 0.0, 'peak_pct': 0.0, 'rss_mb': 0.0})
                e['cpu_s'] += d
                e['peak_pct'] = max(e['peak_pct'], 100.0 * d / dt)
        for pid, (name, _cpu, rss) in snap.items():
            e = self.by_name.setdefault(name, {'cpu_s': 0.0, 'peak_pct': 0.0, 'rss_mb': 0.0})
            e['rss_mb'] = max(e['rss_mb'], rss)
        self.prev = snap

    def top(self, n=10):
        rows = []
        for name, e in self.by_name.items():
            mean = 100.0 * e['cpu_s'] / self.elapsed if self.elapsed > 0 else 0.0
            rows.append({'name': name, 'mean_pct': round(mean, 1),
                         'peak_pct': round(e['peak_pct'], 1), 'rss_mb_max': round(e['rss_mb'], 1)})
        rows.sort(key=lambda r: (-r['mean_pct'], -r['peak_pct']))
        return rows[:n]


def summarize_csv_columns(rows, skip=('timestamp',)):
    """{column: [floats]} of a CSV's numeric columns (blank cells dropped)."""
    cols = {}
    for row in rows:
        for k, v in row.items():
            if k in skip or v in (None, ''):
                continue
            try:
                cols.setdefault(k, []).append(float(v))
            except ValueError:
                pass
    return cols


def gpu_summary_jetson(rows):
    """jetson_gpu_sampler.py rows -> load median/max, clock range, rail power max, temps max."""
    cols = summarize_csv_columns(rows)
    load = cols.get('gpu_load_percent', [])
    out = {'source': 'jetson_gpu_sampler', 'samples': len(rows),
           'load_median_pct': round(median(load), 1) if load else None,
           'load_max_pct': round(max(load), 1) if load else None,
           'freq_mhz_min': min(cols['gpu_freq_mhz']) if cols.get('gpu_freq_mhz') else None,
           'freq_mhz_max': max(cols['gpu_freq_mhz']) if cols.get('gpu_freq_mhz') else None,
           'rails_mw_max': {k[:-3]: max(v) for k, v in cols.items() if k.endswith('_mw')},
           'rails_mw_median': {k[:-3]: median(v) for k, v in cols.items() if k.endswith('_mw')},
           'temps_c_max': {k[5:-2]: max(v) for k, v in cols.items()
                           if k.startswith('temp_') and k.endswith('_c')}}
    return out


def gpu_summary_nvidia(rows):
    """nvidia-smi rows (utilization.gpu, temperature.gpu, power.draw) -> the same shape."""
    cols = summarize_csv_columns(rows)
    load = cols.get('util_pct', [])
    return {'source': 'nvidia-smi', 'samples': len(rows),
            'load_median_pct': round(median(load), 1) if load else None,
            'load_max_pct': round(max(load), 1) if load else None,
            'rails_mw_max': {'gpu_board': round(1000 * max(cols['power_w']))} if cols.get('power_w') else {},
            'temps_c_max': {'gpu': max(cols['temp_c'])} if cols.get('temp_c') else {}}


TEGRA_TEMP = re.compile(r'([A-Za-z0-9_]+)@(-?\d+(?:\.\d+)?)C')
TEGRA_CPU = re.compile(r'CPU\s+\[([^\]]+)\]')
TEGRA_CORE = re.compile(r'(\d+)%@(\d+)')


def tegrastats_extra(lines, cpu_caps_mhz=None):
    """What tegrastats_summary.py does not report: temperature maxima, the per-core
    clock range, and busy cores running below their frequency cap (throttling).

    cpu_caps_mhz: per-core scaling_max_freq in MHz (the nvpmodel cap), if known."""
    temps = {}
    freq = {}
    throttled = 0
    samples = 0
    for line in lines:
        if 'GR3D_FREQ' not in line:
            continue
        samples += 1
        for name, val in TEGRA_TEMP.findall(line):
            temps[name] = max(temps.get(name, -1e9), float(val))
        m = TEGRA_CPU.search(line)
        if not m:
            continue
        for i, (load, mhz) in enumerate(TEGRA_CORE.findall(m.group(1))):
            load, mhz = float(load), float(mhz)
            lo, hi = freq.get(i, (mhz, mhz))
            freq[i] = (min(lo, mhz), max(hi, mhz))
            cap = cpu_caps_mhz[i] if cpu_caps_mhz and i < len(cpu_caps_mhz) else None
            if cap and load >= BUSY_CORE_PCT and mhz < FREQ_THROTTLE_RATIO * cap:
                throttled += 1
    return {'samples': samples, 'temps_c_max': temps,
            'cpu_freq_mhz_range': {f'cpu{i}': list(v) for i, v in sorted(freq.items())},
            'busy_core_below_cap_samples': throttled}


def throttling(temps_max, trips, oc_delta, tegra_extra=None):
    """Reasons to believe the board throttled: a zone at or over its passive trip, an
    over-current event, or (tegrastats) a busy core below its frequency cap."""
    reasons = []
    for zone, t in sorted(temps_max.items()):
        trip = trips.get(zone)
        if trip is not None and t >= trip:
            reasons.append(f'{zone} reached {t:.1f} C (passive trip {trip:.1f} C)')
    for name, d in sorted(oc_delta.items()):
        if d > 0:
            reasons.append(f'{d} over-current event(s) on {name}')
    if tegra_extra and tegra_extra.get('busy_core_below_cap_samples'):
        reasons.append(f'{tegra_extra["busy_core_below_cap_samples"]} core-samples busy '
                       f'(>= {BUSY_CORE_PCT:g} %) below {FREQ_THROTTLE_RATIO:.0%} of the clock cap')
    return {'seen': bool(reasons), 'reasons': reasons}


def system_verdicts(cpu, mem, throttle, gpu):
    out = []
    if cpu and cpu['cores']:
        hot = [c['core'] for c in cpu['cores'] if c['median_pct'] >= CORE_MEDIAN_WARN_PCT]
        out.append((WARN if hot else OK, 'cpu',
                    f'total median {cpu["total_median_pct"]:.0f} %, peak {cpu["total_peak_pct"]:.0f} %'
                    + (f' -- {hot} busy >= {CORE_MEDIAN_WARN_PCT:g} % half the time' if hot else '')))
    if mem and mem.get('peak_pct') is not None:
        grew = mem.get('swap_growth_mb', 0.0)
        lvl = WARN if mem['peak_pct'] >= MEM_WARN_PCT or grew > SWAP_GROWTH_WARN_MB else OK
        out.append((lvl, 'memory', f'peak {mem["peak_used_mb"]:.0f} / {mem["total_mb"]:.0f} MB '
                    f'({mem["peak_pct"]:.0f} %), swap grew {grew:.0f} MB'
                    + (' -- memory pressure: something is being swapped out' if grew > SWAP_GROWTH_WARN_MB else '')))
    if throttle is not None:
        out.append((WARN if throttle['seen'] else OK, 'throttling',
                    '; '.join(throttle['reasons']) if throttle['seen'] else 'none seen'))
    if gpu and gpu.get('load_max_pct') is not None:
        out.append((OK, 'gpu', f'load median {gpu["load_median_pct"]:.0f} %, max {gpu["load_max_pct"]:.0f} %'))
    return out


def render_text(s):
    """summary.json's dict -> summary.txt."""
    w = []
    w.append(f'Lab 2 health, {s["started"]} for {s["duration_s"]:.0f} s, mode {s["mode"]}, '
             f'{s["host"]["machine"]}{" (Jetson)" if s["host"]["jetson"] else ""}')
    if s['host'].get('nvpmodel'):
        w.append(f'power mode: {s["host"]["nvpmodel"]}')
    w.append('')
    w.append('verdict')
    for v in s['verdicts']:
        w.append(f'  {v["level"]:<4}  {v["name"]}: {v["detail"]}')
    w.append('')
    w.append(f'{"topics":<52}{"n":>6} {"median":>7} {"p5":>6} {"min":>6} {"max gap":>8}  age at end')

    def f(x, fmt):
        return format(x, fmt) if x is not None else '-'
    for t, st in s['topics'].items():
        if st is None:
            w.append(f'  {t:<50} (never published)')
            continue
        w.append(f'  {t:<50}{st["n"]:>6} {f(st["median_hz"], "6.1f"):>7} {f(st["p5_hz"], "6.1f"):>6} '
                 f'{f(st["min_hz"], "6.1f"):>6} {f(st["max_gap_s"], "7.2f"):>8}  {f(st["age_at_end_s"], ".2f")}')
    cpu = s.get('cpu')
    if cpu and cpu['cores']:
        w.append('')
        w.append(f'cpu  total median {cpu["total_median_pct"]} %, peak {cpu["total_peak_pct"]} %; '
                 f'load1 median {s["load"]["load1_median"]}, max {s["load"]["load1_max"]}')
        w.append('  per core median/peak %:')
        cells = [f'{c["core"][3:]:>2}:{c["median_pct"]:3.0f}/{c["peak_pct"]:3.0f}' for c in cpu['cores']]
        w += ['    ' + '  '.join(cells[i:i + 8]) for i in range(0, len(cells), 8)]
    mem = s.get('memory')
    if mem:
        w.append(f'memory  peak {mem["peak_used_mb"]:.0f} / {mem["total_mb"]:.0f} MB ({mem["peak_pct"]:.0f} %), '
                 f'swap peak {mem["swap_peak_mb"]:.0f} MB (grew {mem.get("swap_growth_mb", 0):.0f} MB)')
    if s.get('processes'):
        w.append('')
        w.append(f'{"top ROS processes":<52}{"mean %":>7} {"peak %":>7} {"RSS MB":>8}')
        for p in s['processes']:
            w.append(f'  {p["name"][:50]:<50}{p["mean_pct"]:>7.1f} {p["peak_pct"]:>7.1f} {p["rss_mb_max"]:>8.0f}')
    gpu = s.get('gpu')
    if gpu:
        w.append('')
        w.append(f'gpu ({gpu["source"]}, {gpu["samples"]} samples)  load median {gpu["load_median_pct"]} %, '
                 f'max {gpu["load_max_pct"]} %'
                 + (f', clock {gpu["freq_mhz_min"]:.0f}-{gpu["freq_mhz_max"]:.0f} MHz'
                    if gpu.get('freq_mhz_min') is not None else ''))
        if gpu.get('rails_mw_max'):
            w.append('  power max mW: ' + ', '.join(f'{k} {v:.0f}' for k, v in gpu['rails_mw_max'].items()))
    if s.get('temperatures_c_max'):
        w.append('temperatures max C: ' + ', '.join(
            f'{k} {v:.1f}' for k, v in sorted(s['temperatures_c_max'].items())))
    th = s.get('throttling')
    if th is not None:
        w.append(f'throttling: {"; ".join(th["reasons"]) if th["seen"] else "none seen"}')
    if s.get('tegrastats'):
        w.append('')
        w.append('tegrastats_summary.py:')
        w += ['  ' + line for line in s['tegrastats'].get('summary', '').splitlines()]
        rng = s['tegrastats'].get('cpu_freq_mhz_range')
        if rng:
            w.append('  cpu clock MHz min-max: ' + '  '.join(f'{k[3:]}:{a:.0f}-{b:.0f}' for k, (a, b) in rng.items()))
    if s.get('notes'):
        w.append('')
        w.append('notes')
        w += [f'  - {n}' for n in s['notes']]
    w.append('')
    w.append('files: ' + ', '.join(s.get('files', [])))
    return '\n'.join(w) + '\n'


# ------------------------------------------------------------------ the specs

def _load_yaml(path):
    try:
        import yaml
        with open(path) as fh:
            return yaml.safe_load(fh)
    except Exception:
        return None


def monitor_params(entries):
    """{topic: {warn_rate, error_rate, timeout}} from topics-waypoint.yaml, plus
    '*' for the common timeout (the first entry's)."""
    out = {}
    for e in entries or []:
        a = e.get('args', {})
        if 'topic' in a and a['topic'] != '/tf':
            out[a['topic']] = {k: float(a.get(k, DEFAULT_MONITOR[k])) for k in DEFAULT_MONITOR}
    out['*'] = next(iter(out.values()), dict(DEFAULT_MONITOR))
    return out


def build_specs(rates, monitor, error_age):
    """The monitored topics with their thresholds.

    rates: waypoint_preflight.RATES, (topic, label, fail_hz, warn_hz, modes).
    monitor: monitor_params(); error_age: localization_monitor's, seconds.
    """
    pre = {r[0]: r for r in rates}

    def from_pre(topic, label=None, alias=None):
        t, lab, fail, warn, modes = pre[alias or topic]
        timeout = monitor.get(topic, monitor['*'])['timeout']
        return {'topic': topic, 'label': label or lab, 'kind': 'rate', 'fail_hz': fail,
                'warn_hz': warn, 'timeout_s': timeout, 'modes': modes,
                'source': f'preflight {fail:g}/{warn:g} Hz, gap > {timeout:g} s'}

    def from_monitor(topic, label):
        m = monitor.get(topic, monitor['*'])
        return {'topic': topic, 'label': label, 'kind': 'rate', 'fail_hz': m['error_rate'],
                'warn_hz': m['warn_rate'], 'timeout_s': m['timeout'], 'modes': ALL_MODES,
                'source': f'topic_state_monitor {m["error_rate"]:g}/{m["warn_rate"]:g} Hz, '
                          f'gap > {m["timeout"]:g} s'}

    image = next(r[0] for r in rates if r[0].endswith('/image'))
    specs = [
        # camera_info is published with every image (image_transport's CameraPublisher);
        # subscribing to the raw image from outside the container would make the ZED
        # serialise every frame for this monitor alone
        from_pre(CAMERA_INFO, 'camera image (its camera_info)', alias=image),
        from_pre('/sensing/imu/imu_data'),
        from_pre('/vehicle/status/velocity_status'),
        {'topic': FIX, 'label': 'marker fixes', 'kind': 'fix', 'warn_hz': FIX_WARN_RATE,
         'fail_gap_s': error_age, 'warn_gap_s': error_age / 2, 'modes': ALL_MODES,
         'source': f'preflight {FIX_WARN_RATE:g}/s, localization_monitor error_age {error_age:g} s'},
        from_pre('/localization/kinematic_state'),
        from_monitor(CONTROL_CMD, 'control_cmd (vehicle_cmd_gate)'),
        from_monitor(PWM, 'actuator PWM'),
    ]
    return specs


def load_specs():
    sys.path.insert(0, str(HERE))
    from waypoint_preflight import RATES  # rclpy is imported there: runtime only
    monitor = monitor_params(_load_yaml(MONITOR_TOPICS))
    loc = _load_yaml(LOC_MONITOR) or {}
    error_age = float(loc.get('/**', {}).get('ros__parameters', {}).get('error_age', DEFAULT_ERROR_AGE))
    return build_specs(RATES, monitor, error_age)


# ------------------------------------------------------------------ sampling

def read(path):
    try:
        with open(path) as fh:
            return fh.read()
    except Exception:
        return None


def thermal_zones():
    """[(type, temp_path, passive_trip_c or None)] for readable zones."""
    zones = []
    for z in sorted(glob.glob('/sys/class/thermal/thermal_zone*')):
        kind = (read(f'{z}/type') or '').strip()
        t = (read(f'{z}/temp') or '').strip()
        if not kind or not t.lstrip('-').isdigit():
            continue
        trip = None
        for tp in glob.glob(f'{z}/trip_point_*_type'):
            if (read(tp) or '').strip() == 'passive':
                v = (read(tp.replace('_type', '_temp')) or '').strip()
                if v.lstrip('-').isdigit() and int(v) > 0:
                    trip = min(trip, int(v) / 1000.0) if trip else int(v) / 1000.0
        zones.append((kind, f'{z}/temp', trip))
    return zones


def oc_counters():
    """{hwmonN/ocK_event_cnt: count}: the Jetson's over-current throttle events."""
    out = {}
    for p in glob.glob('/sys/class/hwmon/hwmon*/oc*_event_cnt'):
        v = (read(p) or '').strip()
        if v.isdigit():
            out[f'{Path(p).parent.name}/{Path(p).name}'] = int(v)
    return out


CLK_TCK = os.sysconf('SC_CLK_TCK')
PAGE_MB = os.sysconf('SC_PAGE_SIZE') / 1048576.0


def proc_snapshot():
    """{pid: (name, cpu_seconds, rss_mb)} of the ROS processes."""
    snap = {}
    me = os.getpid()
    for pid in os.listdir('/proc'):
        if not pid.isdigit() or int(pid) == me:
            continue
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as fh:
                cmd = fh.read().replace(b'\0', b' ').decode('utf-8', 'replace')
            if not is_ros_process(cmd):
                continue
            with open(f'/proc/{pid}/stat') as fh:
                stat = fh.read()
            comm = stat[stat.index('(') + 1:stat.rindex(')')]
            f = stat.rsplit(') ', 1)[1].split()
            cpu = (int(f[11]) + int(f[12])) / CLK_TCK
            rss = int(f[21]) * PAGE_MB
        except Exception:
            continue
        snap[int(pid)] = (process_name(comm, cmd), cpu, rss)
    return snap


class SystemSampler(threading.Thread):
    """/proc and thermal zones, every `interval` seconds, until stop is set."""

    def __init__(self, interval, stop):
        super().__init__(daemon=True)
        self.interval = interval
        self.stop = stop
        self.cpu = []
        self.load = []
        self.mem = []
        self.temps = {}
        self.zones = thermal_zones()
        self.procs = ProcTracker()

    def run(self):
        prev = parse_proc_stat(read('/proc/stat') or '')
        last = time.monotonic()
        self.procs.update(proc_snapshot(), 0.0)
        while not self.stop.wait(self.interval):
            now = time.monotonic()
            cur = parse_proc_stat(read('/proc/stat') or '')
            self.cpu.append(cpu_busy(prev, cur))
            prev = cur
            la = (read('/proc/loadavg') or '0 0 0').split()
            self.load.append(float(la[0]))
            self.mem.append(parse_meminfo(read('/proc/meminfo') or ''))
            for kind, path, _trip in self.zones:
                v = (read(path) or '').strip()
                if v.lstrip('-').isdigit():
                    self.temps.setdefault(kind, []).append(int(v) / 1000.0)
            self.procs.update(proc_snapshot(), now - last)
            last = now

    def memory(self):
        if not self.mem:
            return None
        used = [m['MemTotal'] - m['MemAvailable'] for m in self.mem if 'MemAvailable' in m]
        swap = [m['SwapTotal'] - m['SwapFree'] for m in self.mem if 'SwapFree' in m]
        total = self.mem[0].get('MemTotal', 0)
        return {'peak_used_mb': round(max(used), 1), 'median_used_mb': round(median(used), 1),
                'total_mb': round(total, 1), 'peak_pct': round(100 * max(used) / total, 1) if total else None,
                'swap_peak_mb': round(max(swap), 1) if swap else 0.0,
                'swap_growth_mb': round(max(swap) - swap[0], 1) if swap else 0.0}


def start_child(cmd, out_path):
    fh = open(out_path, 'w')
    # a session of its own: the terminal's Ctrl-C reaches this script, which then
    # stops the children itself and still writes the summary
    return subprocess.Popen(cmd, stdout=fh, stderr=subprocess.STDOUT, start_new_session=True), fh


def stop_child(proc, fh, grace=3.0):
    if proc.poll() is None:
        try:
            os.killpg(proc.pid, signal.SIGINT)
            proc.wait(timeout=grace)
        except (subprocess.TimeoutExpired, ProcessLookupError):
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            proc.wait()
    fh.close()


def cpu_caps_mhz():
    caps = []
    for p in sorted(glob.glob('/sys/devices/system/cpu/cpu[0-9]*/cpufreq/scaling_max_freq'),
                    key=lambda s: int(re.search(r'cpu(\d+)/', s).group(1))):
        v = (read(p) or '').strip()
        caps.append(int(v) / 1000.0 if v.isdigit() else None)
    return caps


def newest_bag_since(bag_dir, since_epoch):
    """The newest lab2_* bag directory created at or after since_epoch, or None."""
    cands = [p for p in Path(bag_dir).glob('lab2_*') if p.is_dir() and p.stat().st_mtime >= since_epoch - 5]
    return max(cands, key=lambda p: p.stat().st_mtime) if cands else None


def config_bag_dir():
    """bag_dir from the Lab 2 settings (~/lab2/lab2.yaml over the defaults), else rosbags."""
    try:
        sys.path.insert(0, str(HERE))
        import lab2_config
        args = lab2_config.plan('record', [])
        return args[args.index('--out-dir') + 1]
    except Exception:
        return 'rosbags'


# ------------------------------------------------------------------ main

def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument('--duration', type=float, default=0.0, help='stop after S seconds (default: Ctrl-C)')
    ap.add_argument('--interval', type=float, default=1.0, help='CPU/GPU sampling interval [s]')
    ap.add_argument('--out', help='output directory (default rosbags/health/<date>_<time>)')
    ap.add_argument('--with-run', nargs='?', const='', metavar='BAG_DIR',
                    help='file the result under BAG_DIR/run_logs/<newest lab2_* bag>/ '
                         '(default BAG_DIR: the Lab 2 settings\' bag_dir)')
    ap.add_argument('--kernel', action='store_true',
                    help='one kernel_cpu_report.sh snapshot at the end (about 10 s more)')
    ap.add_argument('--mode', choices=['auto', 'vehicle', 'bench', 'mock'], default='auto')
    args = ap.parse_args()

    started = datetime.now()
    start_epoch = time.time()
    stamp = started.strftime('%Y-%m-%d_%H-%M-%S')
    out = Path(args.out) if args.out else REPO / 'rosbags' / 'health' / stamp
    out.mkdir(parents=True, exist_ok=True)
    notes = []

    stop = threading.Event()

    def on_signal(signum, _frame):
        if stop.is_set():
            raise KeyboardInterrupt          # a second Ctrl-C: stop waiting
        stop.set()
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, on_signal)

    specs = load_specs()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from rosidl_runtime_py.utilities import get_message

    # ---- children
    children = []
    jetson = read(JETSON_GPU_LOAD) is not None
    gpu_csv = None
    if jetson:
        gpu_csv = out / 'gpu.csv'
        children.append(start_child([sys.executable, str(GPU_SAMPLER), '-i', str(args.interval),
                                     '-o', str(gpu_csv)], out / 'gpu_sampler.log'))
    elif shutil.which('nvidia-smi'):
        gpu_csv = out / 'gpu_nvidia.csv'
        children.append(start_child(['nvidia-smi', '--query-gpu=utilization.gpu,temperature.gpu,power.draw',
                                     '--format=csv,noheader,nounits', '-lms', str(int(args.interval * 1000))],
                                    gpu_csv))
        notes.append('not a Jetson: jetson_gpu_sampler.py skipped, GPU from nvidia-smi')
    else:
        notes.append('not a Jetson and no nvidia-smi: no GPU sampling')
    tegra_log = None
    if shutil.which('tegrastats'):
        tegra_log = out / 'tegrastats.log'
        children.append(start_child(['tegrastats', '--interval', str(int(args.interval * 1000))], tegra_log))
    else:
        notes.append('tegrastats not installed: no tegrastats log (Jetson only)')
    oc0 = oc_counters()

    sampler = SystemSampler(args.interval, stop)
    sampler.start()

    # ---- topics
    from rclpy.executors import ExternalShutdownException
    from rclpy.signals import SignalHandlerOptions
    # rclpy's own SIGINT handler would shut the context down under the loop: keep ours
    rclpy.init(args=['--ros-args', '--disable-rosout-logs'], signal_handler_options=SignalHandlerOptions.NO)
    node = Node('lab2_health')
    qos = QoSProfile(depth=50, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    times = {s['topic']: [] for s in specs}
    sub_at = {}
    nodes_seen = set()

    def subscribe_new():
        types = dict(node.get_topic_names_and_types())
        for s in specs:
            t = s['topic']
            if t in sub_at or t not in types:
                continue
            try:
                cls = get_message(types[t][0])
            except Exception as e:
                notes.append(f'{t}: cannot load {types[t][0]} ({e})')
                sub_at[t] = None
                continue
            lst = times[t]
            node.create_subscription(cls, t, lambda _m, lst=lst: lst.append(time.monotonic()), qos, raw=True)
            sub_at[t] = time.monotonic()
        nodes_seen.update(f'{ns.rstrip("/")}/{n}' for n, ns in node.get_node_names_and_namespaces())

    t0 = time.monotonic()
    print(f'lab2 health -> {out}', flush=True)
    print('sampling topics, CPU, memory' + (', GPU' if gpu_csv else '') + (', tegrastats' if tegra_log else '')
          + (f' for {args.duration:g} s' if args.duration else '; Ctrl-C to stop and summarise'), flush=True)
    next_scan = 0.0
    next_status = time.monotonic() + 10.0
    try:
        while not stop.is_set():
            now = time.monotonic()
            if args.duration and now - t0 >= args.duration:
                break
            if now >= next_scan:
                subscribe_new()
                next_scan = now + 1.0
            if now >= next_status:
                live = ', '.join(f'{s["label"].split(" (")[0]} '
                                 f'{sum(1 for x in times[s["topic"]] if x > now - 5) / 5:.0f}'
                                 for s in specs if s['topic'] in sub_at)
                print(f'  {now - t0:5.0f} s  Hz(5 s): {live or "no topics yet"}', flush=True)
                next_status = now + 10.0
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    t_end = time.monotonic()
    stop.set()
    print('stopping: summarising ...', flush=True)
    sampler.join(timeout=5)
    for proc, fh in children:
        stop_child(proc, fh)
    oc1 = oc_counters()

    names = nodes_seen
    if args.mode != 'auto':
        mode = args.mode
    elif '/vehicle_bench_plant' in names:
        mode = 'mock' if '/mock_aruco_camera' in names else 'bench'
    else:
        mode = 'vehicle'
    if len(names) <= 1:
        notes.append('no ROS nodes seen: is the stack up, on this ROS_DOMAIN_ID and RMW?')
    node.destroy_node()
    rclpy.try_shutdown()

    # ---- summarise
    verdicts = []
    topics = {}
    t_stats = t_end
    stopped = stack_stopped_at(times, t_end)
    if stopped is not None:
        t_stats = stopped + 0.001
        verdicts.append({'level': WARN, 'name': 'stack',
                         'detail': f'every topic stopped {t_end - stopped:.1f} s before the monitor did '
                                   '(the stack shut down?): rates and gaps are up to that moment'})
    for s in specs:
        t = s['topic']
        st = rate_stats(times[t], sub_at[t], t_stats) if sub_at.get(t) is not None else None
        topics[t] = st
        v = topic_verdict(s, st, mode)
        if v is None:
            notes.append(f'{s["label"]}: not expected in mode {mode}')
        else:
            verdicts.append({'level': v[0], 'name': 'rate', 'detail': v[1], 'thresholds': s['source']})

    cpu = cpu_summary(sampler.cpu)
    mem = sampler.memory()
    temps = {k: max(v) for k, v in sampler.temps.items()}
    trips = {k: trip for k, _p, trip in sampler.zones if trip is not None}
    gpu = None
    if gpu_csv and gpu_csv.exists():
        with open(gpu_csv) as fh:
            if jetson:
                gpu = gpu_summary_jetson(list(csv.DictReader(fh)))
            else:
                rows = [dict(zip(('util_pct', 'temp_c', 'power_w'), [c.strip() for c in r]))
                        for r in csv.reader(fh) if len(r) >= 3]
                gpu = gpu_summary_nvidia(rows)
        for k, v in gpu.get('temps_c_max', {}).items():
            temps.setdefault(k if jetson else f'gpu ({gpu["source"]})', v)
    tegra = None
    if tegra_log and tegra_log.exists():
        lines = tegra_log.read_text(errors='replace').splitlines()
        tegra = tegrastats_extra(lines, cpu_caps_mhz())
        r = subprocess.run([sys.executable, str(TEGRASTATS_SUMMARY), str(tegra_log), '--skip-seconds', '0'],
                           capture_output=True, text=True)
        tegra['summary'] = (r.stdout or r.stderr).strip()
        for k, v in tegra['temps_c_max'].items():
            temps.setdefault(f'{k} (tegrastats)', v)
    oc_delta = {k: oc1.get(k, v) - v for k, v in oc0.items()}
    throttle = throttling(temps, trips, oc_delta, tegra)
    if not jetson:
        notes.append('throttling from thermal-zone passive trips only (Jetson adds over-current '
                     'counters and tegrastats clocks)')
    for lvl, name, detail in system_verdicts(cpu, mem, throttle, gpu):
        verdicts.append({'level': lvl, 'name': name, 'detail': detail})
    la = sampler.load

    nvp = None
    if jetson and shutil.which('nvpmodel'):
        try:
            r = subprocess.run(['nvpmodel', '-q'], capture_output=True, text=True, timeout=5)
            nvp = ' '.join(r.stdout.split()) or None
        except Exception:
            pass

    files = sorted(p.name for p in out.iterdir())
    if args.kernel:
        print('kernel_cpu_report.sh (about 10 s) ...', flush=True)
        try:
            r = subprocess.run(['bash', str(KERNEL_REPORT), '2'], capture_output=True, text=True, timeout=60)
            text = re.sub(r'\x1b\[[0-9;]*m', '', r.stdout + r.stderr)
            (out / 'kernel_cpu_report.txt').write_text(text)
            files.append('kernel_cpu_report.txt')
        except (subprocess.TimeoutExpired, KeyboardInterrupt):
            notes.append('kernel_cpu_report.sh did not finish')

    summary = {
        'started': started.isoformat(timespec='seconds'), 'duration_s': round(t_end - t0, 1),
        'mode': mode, 'host': {'machine': platform.machine(), 'node': platform.node(),
                               'jetson': jetson, 'nvpmodel': nvp},
        'verdicts': verdicts, 'topics': topics,
        'cpu': cpu, 'load': {'load1_median': median(la), 'load1_max': max(la) if la else None},
        'memory': mem, 'processes': sampler.procs.top(10), 'gpu': gpu,
        'temperatures_c_max': temps, 'passive_trips_c': trips, 'throttling': throttle,
        'tegrastats': tegra, 'notes': notes, 'files': files + ['summary.txt', 'summary.json'],
    }

    if args.with_run is not None:
        bag_dir = args.with_run or config_bag_dir()
        bag = newest_bag_since(bag_dir, start_epoch)
        if bag is None:
            notes.append(f'--with-run: no lab2_* bag in {bag_dir} started after this monitor; '
                         f'left in {out}')
        else:
            dest = bag.parent / 'run_logs' / bag.name / f'health_{stamp}'
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.move(str(out), str(dest))
            out = dest
            summary['bag'] = str(bag)
    summary['out_dir'] = str(out)

    (out / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    text = render_text(summary)
    (out / 'summary.txt').write_text(text)
    print()
    print(text, end='')
    print(f'-> {out}/summary.txt, summary.json')
    return 1 if any(v['level'] == FAIL for v in verdicts) else 0


if __name__ == '__main__':
    sys.exit(main())
