"""health.py's summarising logic, without ROS or a stack:
python3 -m pytest scripts/testing/lab2/test_health.py"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import health as h  # noqa: E402

RATES = [  # the shape of waypoint_preflight.RATES
    ('/sensing/camera/zedxm/rgb/color/rect/image', 'camera image', 8.0, 12.0, ('vehicle', 'mock')),
    ('/sensing/imu/imu_data', 'IMU', 20.0, 30.0, ('vehicle', 'bench', 'mock')),
    ('/vehicle/status/velocity_status', 'velocity_status', 10.0, 18.0, ('vehicle', 'bench', 'mock')),
    ('/localization/kinematic_state', 'kinematic_state', 20.0, 40.0, ('vehicle', 'bench', 'mock')),
]
MONITOR = [{'args': {'topic': '/vehicle/status/velocity_status', 'warn_rate': 5.0,
                     'error_rate': 1.0, 'timeout': 1.0}},
           {'args': {'topic': '/control/command/control_cmd', 'warn_rate': 5.0,
                     'error_rate': 1.0, 'timeout': 1.0}},
           {'args': {'topic': '/tf', 'warn_rate': 5.0, 'error_rate': 1.0, 'timeout': 1.0}}]


def specs():
    return {s['topic']: s for s in h.build_specs(RATES, h.monitor_params(MONITOR), 5.0)}


def steady(hz, start, end):
    n = int((end - start) * hz)
    return [start + i / hz for i in range(n)]


def test_percentile_interpolates_and_handles_edges():
    assert h.percentile([], 5) is None
    assert h.percentile([3], 5) == 3.0
    assert h.percentile([0, 10], 50) == 5.0
    assert h.percentile(list(range(101)), 5) == 5.0


def test_steady_stream_rates_and_gaps():
    st = h.rate_stats(steady(50, 0.0, 10.0), 0.0, 10.0)
    assert st['n'] == 500
    assert st['median_hz'] == 50.0 and st['min_hz'] == 50.0
    assert abs(st['max_gap_s'] - 0.02) < 1e-6


def test_a_stream_that_dies_shows_as_gap_and_zero_hz_bins():
    st = h.rate_stats(steady(50, 0.0, 4.0), 0.0, 10.0)
    assert st['min_hz'] == 0.0 and st['p5_hz'] == 0.0
    assert abs(st['max_gap_s'] - 6.02) < 1e-6
    assert abs(st['age_at_end_s'] - 6.02) < 1e-6


def test_discovery_delay_is_not_a_gap():
    st = h.rate_stats(steady(50, 3.0, 10.0), 0.0, 10.0)
    assert st['max_gap_s'] < 0.05
    assert st['median_hz'] == 50.0


def test_no_messages():
    st = h.rate_stats([], 0.0, 10.0)
    assert st['n'] == 0 and st['median_hz'] is None


def test_specs_reuse_preflight_and_monitor_thresholds():
    s = specs()
    imu = s['/sensing/imu/imu_data']
    assert (imu['fail_hz'], imu['warn_hz'], imu['timeout_s']) == (20.0, 30.0, 1.0)
    cam = s[h.CAMERA_INFO]
    assert (cam['fail_hz'], cam['warn_hz'], cam['modes']) == (8.0, 12.0, ('vehicle', 'mock'))
    cmd = s[h.CONTROL_CMD]
    assert (cmd['fail_hz'], cmd['warn_hz'], cmd['timeout_s']) == (1.0, 5.0, 1.0)
    fix = s[h.FIX]
    assert (fix['warn_hz'], fix['fail_gap_s']) == (2.0, 5.0)
    assert s[h.PWM]['timeout_s'] == 1.0          # not monitored: the common timeout


def test_imu_verdicts_follow_preflight():
    s = specs()['/sensing/imu/imu_data']
    lvl, _ = h.topic_verdict(s, h.rate_stats(steady(35, 0, 20), 0, 20), 'vehicle')
    assert lvl == h.OK
    lvl, text = h.topic_verdict(s, h.rate_stats(steady(25, 0, 20), 0, 20), 'vehicle')
    assert lvl == h.WARN and 'under 30' in text
    lvl, _ = h.topic_verdict(s, h.rate_stats(steady(15, 0, 20), 0, 20), 'vehicle')
    assert lvl == h.FAIL


def test_a_gap_over_the_monitor_timeout_fails():
    s = specs()['/vehicle/status/velocity_status']
    times = steady(20, 0, 8) + steady(20, 9.5, 20)
    lvl, text = h.topic_verdict(s, h.rate_stats(times, 0, 20), 'vehicle')
    assert lvl == h.FAIL and 'monitor timeout' in text


def test_camera_not_expected_on_the_plain_bench():
    s = specs()[h.CAMERA_INFO]
    assert h.topic_verdict(s, None, 'bench') is None
    lvl, text = h.topic_verdict(s, None, 'vehicle')
    assert lvl == h.FAIL and 'no publisher' in text


def test_fix_gaps_against_error_age():
    s = specs()[h.FIX]
    ok = h.rate_stats(steady(10, 0, 20), 0, 20)
    assert h.topic_verdict(s, ok, 'vehicle')[0] == h.OK
    warn = h.rate_stats(steady(10, 0, 8) + steady(10, 11, 20), 0, 20)
    assert h.topic_verdict(s, warn, 'vehicle')[0] == h.WARN
    fail = h.rate_stats(steady(10, 0, 5) + steady(10, 11, 20), 0, 20)
    assert h.topic_verdict(s, fail, 'vehicle')[0] == h.FAIL


def test_proc_stat_and_cpu_busy():
    a = h.parse_proc_stat('cpu  100 0 100 800 0 0 0 0 0 0\ncpu0 50 0 50 400 0 0 0 0\n')
    b = h.parse_proc_stat('cpu  200 0 200 900 0 0 0 0 0 0\ncpu0 150 0 50 400 0 0 0 0\n')
    busy = h.cpu_busy(a, b)
    assert abs(busy['cpu'] - 200 / 300 * 100) < 1e-9
    assert busy['cpu0'] == 100.0


def test_cpu_summary_orders_cores_numerically():
    samples = [{'cpu': 10, 'cpu0': 5, 'cpu10': 50, 'cpu2': 1}, {'cpu': 30, 'cpu0': 15, 'cpu10': 100, 'cpu2': 3}]
    s = h.cpu_summary(samples)
    assert [c['core'] for c in s['cores']] == ['cpu0', 'cpu2', 'cpu10']
    assert s['cores'][2] == {'core': 'cpu10', 'median_pct': 75.0, 'peak_pct': 100.0}
    assert s['total_peak_pct'] == 30


def test_meminfo():
    m = h.parse_meminfo('MemTotal: 2048 kB\nMemFree: 1 kB\nMemAvailable: 1024 kB\nSwapTotal: 0 kB\nSwapFree: 0 kB\n')
    assert m == {'MemTotal': 2.0, 'MemAvailable': 1.0, 'SwapTotal': 0.0, 'SwapFree': 0.0}


def test_process_names():
    assert h.process_name('python3', '/usr/bin/python3 x --ros-args -r __node:=actuator_node '
                          '-r __ns:=/autosdv') == '/autosdv/actuator_node'
    assert h.process_name('component_conta', 'component_container --ros-args -r __node:=zed_container') \
        == 'zed_container'
    assert h.process_name('play_launch', 'play_launch launch x') == 'play_launch'
    assert h.process_name('autoware_ekf_lo', '/opt/autoware/1.5.0/lib/autoware_ekf_localizer/'
                          'autoware_ekf_localizer_node --ros-args --params-file x') == 'autoware_ekf_localizer_node'
    assert h.process_name('python3', '/usr/bin/python3 /opt/ros/humble/bin/ros2 bag record -a') == 'ros2'
    assert h.process_name('ros2', 'ros2 bag record -a') == 'ros2 bag'
    assert h.is_ros_process('/opt/ros/humble/lib/rclcpp_components/component_container --ros-args')
    assert not h.is_ros_process('/usr/bin/bash')


def test_proc_tracker_mean_and_peak():
    t = h.ProcTracker()
    t.update({1: ('a', 0.0, 10.0), 2: ('b', 0.0, 5.0)}, 0.0)
    t.update({1: ('a', 1.0, 12.0), 2: ('b', 0.1, 5.0)}, 1.0)    # a: 100 %
    t.update({1: ('a', 1.5, 11.0), 2: ('b', 0.2, 5.0), 3: ('c', 50.0, 1.0)}, 1.0)
    top = t.top()
    assert top[0] == {'name': 'a', 'mean_pct': 75.0, 'peak_pct': 100.0, 'rss_mb_max': 12.0}
    assert top[1]['name'] == 'b' and top[1]['mean_pct'] == 10.0
    # c appeared in the last snapshot: its 50 s of earlier CPU is not in the window
    assert next(r for r in top if r['name'] == 'c')['mean_pct'] == 0.0


def test_jetson_gpu_csv():
    rows = [{'timestamp': 't', 'gpu_load_percent': '10.0', 'gpu_freq_mhz': '306', 'VDD_GPU_SOC_mw': '4000',
             'temp_cpu-thermal_c': '50.0'},
            {'timestamp': 't', 'gpu_load_percent': '90.0', 'gpu_freq_mhz': '1300', 'VDD_GPU_SOC_mw': '20000',
             'temp_cpu-thermal_c': '61.5'},
            {'timestamp': 't', 'gpu_load_percent': '', 'gpu_freq_mhz': '', 'VDD_GPU_SOC_mw': '',
             'temp_cpu-thermal_c': ''}]
    g = h.gpu_summary_jetson(rows)
    assert g['load_max_pct'] == 90.0 and g['load_median_pct'] == 50.0
    assert g['rails_mw_max'] == {'VDD_GPU_SOC': 20000.0}
    assert g['temps_c_max'] == {'cpu-thermal': 61.5}
    assert (g['freq_mhz_min'], g['freq_mhz_max']) == (306.0, 1300.0)


TEGRA = ('10-16-2026 10:00:00 RAM 5000/30000MB CPU [95%@1100,10%@2201,off,off] '
         'EMC_FREQ 0% GR3D_FREQ 45% cpu@55.2C soc2@50.1C tj@61.0C VDD_GPU_SOC 4000mW/4000mW\n')


def test_tegrastats_extra_temps_clocks_and_throttle():
    e = h.tegrastats_extra([TEGRA, 'garbage\n', TEGRA.replace('61.0C', '63.5C')], cpu_caps_mhz=[2201, 2201])
    assert e['samples'] == 2
    assert e['temps_c_max']['tj'] == 63.5
    assert e['cpu_freq_mhz_range']['cpu0'] == [1100.0, 1100.0]
    assert e['busy_core_below_cap_samples'] == 2     # core 0 at 95 % and half its cap
    e = h.tegrastats_extra([TEGRA], cpu_caps_mhz=[1100, 2201])
    assert e['busy_core_below_cap_samples'] == 0


def test_throttling_reasons():
    t = h.throttling({'cpu-thermal': 99.0, 'gpu-thermal': 50.0}, {'cpu-thermal': 99.0, 'gpu-thermal': 99.0},
                     {'hwmon1/oc1_event_cnt': 0})
    assert t['seen'] and 'cpu-thermal' in t['reasons'][0] and len(t['reasons']) == 1
    t = h.throttling({'x': 40.0}, {}, {'hwmon1/oc2_event_cnt': 3})
    assert t['seen'] and 'over-current' in t['reasons'][0]
    assert h.throttling({'x': 40.0}, {'x': 90.0}, {}) == {'seen': False, 'reasons': []}


def test_system_verdicts():
    cpu = h.cpu_summary([{'cpu': 50, 'cpu0': 95, 'cpu1': 5}] * 3)
    mem = {'peak_used_mb': 950, 'total_mb': 1000, 'peak_pct': 95.0, 'swap_peak_mb': 0}
    v = {name: lvl for lvl, name, _ in h.system_verdicts(cpu, mem, {'seen': False, 'reasons': []}, None)}
    assert v == {'cpu': h.WARN, 'memory': h.WARN, 'throttling': h.OK}
    # swap in use before the run is not a warning; swap that grows during it is
    old = {'peak_used_mb': 100, 'total_mb': 1000, 'peak_pct': 10.0, 'swap_peak_mb': 9000, 'swap_growth_mb': 0}
    assert h.system_verdicts(None, old, None, None)[0][0] == h.OK
    old['swap_growth_mb'] = 500
    assert h.system_verdicts(None, old, None, None)[0][0] == h.WARN


def test_render_text_has_every_section():
    s = {'started': '2026-10-16T10:00:00', 'duration_s': 40.0, 'mode': 'bench',
         'host': {'machine': 'x86_64', 'jetson': False, 'nvpmodel': None},
         'verdicts': [{'level': 'OK', 'name': 'rate', 'detail': 'IMU: fine'}],
         'topics': {'/a': h.rate_stats(steady(10, 0, 10), 0, 10), '/b': None},
         'cpu': h.cpu_summary([{'cpu': 10, 'cpu0': 10}]), 'load': {'load1_median': 1.0, 'load1_max': 2.0},
         'memory': {'peak_used_mb': 1, 'total_mb': 2, 'peak_pct': 50, 'swap_peak_mb': 0},
         'processes': [{'name': 'n', 'mean_pct': 1, 'peak_pct': 2, 'rss_mb_max': 3}],
         'gpu': {'source': 'nvidia-smi', 'samples': 3, 'load_median_pct': 1, 'load_max_pct': 2,
                 'rails_mw_max': {'gpu_board': 100}},
         'temperatures_c_max': {'gpu': 50.0}, 'throttling': {'seen': False, 'reasons': []},
         'tegrastats': None, 'notes': ['n1'], 'files': ['summary.txt']}
    text = h.render_text(s)
    for part in ('verdict', 'OK    rate: IMU: fine', '/b', '(never published)', 'per core', 'memory',
                 'top ROS processes', 'gpu (nvidia-smi', 'temperatures max', 'throttling: none seen', 'n1'):
        assert part in text, part


def test_stack_stopping_is_one_cutoff_not_a_gap_per_topic():
    times = {'/a': steady(50, 0, 30), '/b': steady(20, 2, 30), '/c': []}
    cut = h.stack_stopped_at(times, 33.0)
    assert abs(cut - max(times['/a'] + times['/b'])) < 1e-9
    assert h.rate_stats(times['/a'], 0, cut + 0.001)['max_gap_s'] < 0.05
    assert h.stack_stopped_at(times, 30.2) is None            # still publishing at the stop
    assert h.stack_stopped_at({'/a': []}, 10.0) is None
