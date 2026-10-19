"""lab2_part1_dashboard: the live Lab 2 Part 1 curves and marks, in a Qt window.

Plots exactly what the grader graded: it subscribes to the grader's /lab2/part1/trace
(the trace rows) and /lab2/part1/status (checks, violations, marks), and at the end
reloads trace.csv so a late start loses nothing. Sim time on the x axis.

    speed      target (dashed), measured (velocity_report), true (the plant)
    motor PWM  with neutral and the brake zone [min_pwm, neutral) shaded
    distance to the stop line
    red spans  each failed check's window; grey spans an e-stop (the course has none)
    the table  the checks, live (pending until their window closes), then the marks

Needs a display: without DISPLAY / WAYLAND_DISPLAY it exits with a message (launch
with gui:=false for batch runs). QT_QPA_PLATFORM=offscreen renders without one,
for tests (--screenshot FILE --exit-after S). Ctrl-C, SIGTERM or closing the window
stop it cleanly; with --screenshot FILE (or the screenshot parameter, which the launch file
sets to out_dir/dashboard.png) it saves the window there first.
"""
import argparse
import csv
import json
import os
import signal
import sys
import threading
import traceback

DISPLAY_ERROR = ('lab2_part1_dashboard: no display (DISPLAY and WAYLAND_DISPLAY are unset). '
                 'Run with gui:=false for a batch run, or from a desktop session / VNC.')

TRACE_FIELDS = ('t', 's', 'y', 'target', 'measured', 'speed', 'pwm', 'slip_r', 'kappa_r',
                'emergency')
REDRAW_MS = 500        # at most 2 redraws a second
MAX_POINTS = 2000      # per plotted line (decimated beyond)
TRACE_HISTORY = 3000   # trace messages a late subscriber gets (10 Hz: 300 s), as the grader
# Reference palette (dataviz skill, light mode): series 1-2, secondary ink, critical red
SURFACE, INK, INK_2, GRID = '#fcfcfb', '#0b0b0b', '#52514e', '#e4e3df'
C_TRUE, C_MEASURED, C_TARGET = '#2a78d6', '#eb6834', '#52514e'
C_FAIL, C_PASS, C_PENDING, C_ESTOP, C_BRAKE = '#e34948', '#008300', '#8a8984', '#b9b8b2', '#2a78d6'


def have_display() -> bool:
    if os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'):
        return True
    headless_ok = ('offscreen', 'minimal', 'vnc', 'eglfs', 'linuxfb')
    return os.environ.get('QT_QPA_PLATFORM', '') in headless_ok


def load_trace_csv(path: str) -> dict:
    out = {k: [] for k in TRACE_FIELDS}
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            for k in TRACE_FIELDS:
                out[k].append(float(row[k]))
    return out


def spans(t, flags):
    """[(t0, t1)] where flags is true."""
    out, start = [], None
    for ti, f in zip(t, flags):
        if f and start is None:
            start = ti
        elif not f and start is not None:
            out.append((start, ti))
            start = None
    if start is not None and t:
        out.append((start, t[-1]))
    return out


def fmt_value(c: dict) -> str:
    v = c.get('value')
    if v is None:
        return '—'
    return '0' if abs(v) < 1e-9 else f'{v:.3g}'


class Model:
    """What the ROS thread receives and the Qt thread draws, behind one lock."""

    def __init__(self):
        self.lock = threading.Lock()
        self.data = {k: [] for k in TRACE_FIELDS}
        self.status = None
        self.version = 0
        self.reloaded = None
        self.status_text = None

    def add_rows(self, labels: str, values) -> None:
        fields = labels.split(',')
        width = len(fields)
        if width == 0 or len(values) % width:
            return
        with self.lock:
            for i in range(0, len(values), width):
                row = dict(zip(fields, values[i:i + width]))
                if self.data['t'] and row['t'] <= self.data['t'][-1]:
                    continue
                for k in TRACE_FIELDS:
                    self.data[k].append(float(row.get(k, 0.0)))
            self.version += 1

    def set_status_text(self, text: str) -> bool:
        """Take a status message; False (nothing to redraw) when it is the same as the last."""
        if text == self.status_text:
            return False
        self.status_text = text
        self.set_status(json.loads(text))
        return True

    def set_status(self, status: dict) -> None:
        trace_file = status.get('trace_file')
        reload = None
        if trace_file and trace_file != self.reloaded and os.path.isfile(trace_file):
            try:
                reload = load_trace_csv(trace_file)
            except (OSError, ValueError, KeyError):
                reload = None
        with self.lock:
            self.status = status
            if reload is not None:
                self.data = reload
                self.reloaded = trace_file
            self.version += 1


# Autoware's desktop setup exports QT_QPA_PLATFORMTHEME=qt5ct with a dark style sheet; under
# it the table's items do not paint at all (checked: offscreen, PyQt5 5.15). The dashboard
# drops the platform theme for its own process before the QApplication exists.
QT5CT_NOTE = 'QT_QPA_PLATFORMTHEME'


def light_palette(app, QtGui) -> None:
    """The same light look on any desktop theme (see QT5CT_NOTE for the platform theme)."""
    app.setStyle('Fusion')
    app.setFont(QtGui.QFont('DejaVu Sans', 10))
    pal = QtGui.QPalette()
    for role, color in (('Window', SURFACE), ('WindowText', INK), ('Base', '#ffffff'),
                        ('AlternateBase', '#f4f3ef'), ('Text', INK), ('Button', '#efeee9'),
                        ('ButtonText', INK), ('Highlight', C_TRUE), ('HighlightedText', '#ffffff'),
                        ('ToolTipBase', '#ffffff'), ('ToolTipText', INK)):
        pal.setColor(getattr(QtGui.QPalette, role), QtGui.QColor(color))
    app.setPalette(pal)


def make_node(model: Model):
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Float64MultiArray, String

    class DashboardNode(Node):
        def __init__(self):
            super().__init__('lab2_part1_dashboard')
            # where to save the window on exit; --screenshot wins (a parameter, not an
            # argument, in the launch file: out_dir may hold spaces)
            self.screenshot = self.declare_parameter('screenshot', '').value
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            history = QoSProfile(depth=TRACE_HISTORY, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(String, '/lab2/part1/status', self.on_status, latched)
            self.create_subscription(Float64MultiArray, '/lab2/part1/trace', self.on_trace,
                                     history)

        def on_status(self, msg):
            try:
                model.set_status_text(msg.data)
            except ValueError:
                pass

        def on_trace(self, msg):
            dims = msg.layout.dim
            labels = dims[1].label if len(dims) > 1 else ','.join(TRACE_FIELDS)
            model.add_rows(labels, list(msg.data))

    return DashboardNode()


def build_window(model: Model, QtWidgets, QtCore, QtGui, FigureCanvas, Figure):
    class Dashboard(QtWidgets.QMainWindow):
        def __init__(self):
            super().__init__()
            self.setWindowTitle('Lab 2 Part 1: speed and brake control')
            self.resize(1500, 900)
            central = QtWidgets.QWidget()
            central.setStyleSheet(f'background: {SURFACE}; color: {INK};')
            layout = QtWidgets.QHBoxLayout(central)
            self.fig = Figure(figsize=(10, 8), facecolor=SURFACE, layout='constrained')
            self.canvas = FigureCanvas(self.fig)
            layout.addWidget(self.canvas, 5)
            side = QtWidgets.QVBoxLayout()
            self.header = QtWidgets.QLabel('waiting for the grader (/lab2/part1/status)')
            self.header.setWordWrap(True)
            font = self.header.font()
            font.setPointSize(13)
            self.header.setFont(font)
            side.addWidget(self.header)
            self.table = QtWidgets.QTableWidget(0, 4)
            self.table.setHorizontalHeaderLabels(['check', 'value', 'limit', 'result'])
            self.table.verticalHeader().setVisible(False)
            self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
            self.table.setStyleSheet(
                f'QTableWidget {{ background: #ffffff; color: {INK}; gridline-color: {GRID}; }}'
                f'QHeaderView::section {{ background: #efeee9; color: {INK}; border: 0;'
                ' padding: 4px; font-weight: bold; }')
            header = self.table.horizontalHeader()
            header.setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
            header.setStretchLastSection(True)
            self.table.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
            side.addWidget(self.table, 1)
            self.marks = QtWidgets.QLabel('')
            font = self.marks.font()
            font.setPointSize(18)
            font.setBold(True)
            self.marks.setFont(font)
            side.addWidget(self.marks)
            layout.addLayout(side, 2)
            self.setCentralWidget(central)

            ax_v, ax_p, ax_d = self.fig.subplots(3, 1, sharex=True,
                                                 gridspec_kw={'height_ratios': [3, 2, 1.4]})
            self.axes = (ax_v, ax_p, ax_d)
            for ax in self.axes:
                ax.set_facecolor(SURFACE)
                ax.grid(True, color=GRID, linewidth=0.8)
                ax.tick_params(colors=INK_2, labelsize=9)
                for side_name in ('top', 'right'):
                    ax.spines[side_name].set_visible(False)
                for side_name in ('left', 'bottom'):
                    ax.spines[side_name].set_color(GRID)
            self.l_true, = ax_v.plot([], [], color=C_TRUE, lw=2.0, label='true', zorder=3)
            self.l_measured, = ax_v.plot([], [], color=C_MEASURED, lw=1.2, label='measured',
                                         zorder=4)
            self.l_target, = ax_v.plot([], [], '--', color=C_TARGET, lw=1.2, label='target',
                                       zorder=5)
            ax_v.set_ylabel('speed [m/s]', color=INK_2)
            ax_v.legend(loc='upper left', frameon=False, ncol=3, fontsize=9)
            self.l_pwm, = ax_p.plot([], [], color=C_TRUE, lw=1.5, label='motor PWM')
            ax_p.set_ylabel('motor PWM', color=INK_2)
            self.pwm_levels = None
            self.l_dist, = ax_d.plot([], [], color=C_TRUE, lw=1.5)
            ax_d.axhline(0.0, color=INK_2, lw=1.0)
            ax_d.set_ylabel('to stop line [m]', color=INK_2)
            ax_d.set_xlabel('sim time [s]', color=INK_2)
            self.span_artists = []
            self.span_key = None
            self.drawn_version = -1
            self.stop_line_s = None
            self.done = False
            self.layout_frozen = False
            self.table_key = None

        def refresh(self):
            if self.done:
                return
            with model.lock:
                if model.version == self.drawn_version:
                    return
                self.drawn_version = model.version
                n = len(model.data['t'])
                step = max(1, -(-n // MAX_POINTS))   # decimate: at most MAX_POINTS per line
                idx = list(range(0, n, step))
                if idx and idx[-1] != n - 1:
                    idx.append(n - 1)
                d = {k: [v[i] for i in idx] for k, v in model.data.items()}
                em_full = list(model.data['emergency']), list(model.data['t'])
                status = model.status
                final = bool(status and status.get('marks')
                             and (not status.get('trace_file')
                                  or model.reloaded == status.get('trace_file')))
            if status:
                self.update_status(status)
            t = d['t']
            self.l_target.set_data(t, d['target'])
            self.l_measured.set_data(t, d['measured'])
            self.l_true.set_data(t, d['speed'])
            self.l_pwm.set_data(t, d['pwm'])
            if self.stop_line_s is not None:
                self.l_dist.set_data(t, [self.stop_line_s - s for s in d['s']])
            self.redraw(t, em_full, status)
            self.done = final   # the final status and the full trace are drawn: stop

        def redraw(self, t, em_full, status):
            key = (len(status.get('violations', [])) if status else 0, len(t) // 100)
            if key != self.span_key:
                self.span_key = key
                self.draw_spans(em_full[1], em_full[0], status)
            for ax in self.axes:
                ax.relim()
                ax.autoscale_view()
            if t:
                self.axes[0].set_xlim(t[0], max(t[-1], t[0] + 5.0))
            if self.pwm_levels:
                lv = self.pwm_levels
                self.axes[1].set_ylim(lv['brake_pwm'] - 8, lv['max_pwm'] + 8)
            self.canvas.draw_idle()
            if not self.layout_frozen:
                self.canvas.draw()
                self.fig.set_constrained_layout(False)   # lay out once, not on every draw
                self.layout_frozen = True

        def update_status(self, st):
            if self.pwm_levels is None and st.get('pwm_levels'):
                lv = self.pwm_levels = st['pwm_levels']
                ax = self.axes[1]
                ax.axhspan(lv['min_pwm'], lv['init_pwm'], color=C_BRAKE, alpha=0.12, lw=0,
                           label='brake zone')
                ax.axhline(lv['init_pwm'], color=INK_2, lw=1.0, label='neutral')
                ax.axhline(lv['brake_pwm'], color=INK_2, lw=0.8, ls=':',
                           label='override brake')
                ax.legend(loc='upper left', frameon=False, ncol=4, fontsize=9)
            if st.get('stop_line_s') is not None:
                self.stop_line_s = float(st['stop_line_s'])
            state = st.get('state', '?')
            cal = '' if st.get('limits_calibrated') else '  (limits uncalibrated)'
            self.header.setText(f"<b>{st.get('variant', '')}</b>  {state}  "
                                f"t = {st.get('t', 0.0):.1f} s  segment {st.get('segment', '')}"
                                f'{cal}')
            checks = st.get('checks', {})
            key = json.dumps(checks, sort_keys=True)
            if key == self.table_key:
                checks = {}
            else:
                self.table_key = key
                self.table.setRowCount(len(checks))
            for r, (name, c) in enumerate(checks.items()):
                unit = f" {c['unit']}" if c.get('unit') else ''
                if c.get('pass') is None:
                    verdict, color = 'pending', C_PENDING
                elif c['pass']:
                    verdict, color = 'PASS', C_PASS
                else:
                    verdict, color = 'FAIL', C_FAIL
                cells = (name, fmt_value(c), f"{c['limit']:.3g}{unit}", verdict)
                for col, text in enumerate(cells):
                    item = QtWidgets.QTableWidgetItem(text)
                    item.setForeground(QtGui.QColor(color if col == 3 else INK))
                    item.setFont(QtGui.QFont('DejaVu Sans', 10,
                                             QtGui.QFont.Bold if col == 3 else QtGui.QFont.Normal))
                    self.table.setItem(r, col, item)
            if st.get('marks'):
                m = st['marks']
                if st.get('invalid_reason'):
                    self.marks.setText(f"INVALID: {m['passed']}/{m['total']}\n"
                                       f"{st['invalid_reason']}")
                    self.marks.setStyleSheet(f'color: {C_FAIL};')
                    self.marks.setWordWrap(True)
                else:
                    note = f"\n{st['message']}" if st.get('message') else ''
                    self.marks.setText(f"{m['passed']}/{m['total']} passed{note}")
                    self.marks.setWordWrap(True)
                    self.marks.setStyleSheet(f'color: {INK};')

        def draw_spans(self, t, emergency, status):
            for a in self.span_artists:
                a.remove()
            self.span_artists = []
            for t0, t1 in spans(t, [e > 0.5 for e in emergency]):
                for ax in self.axes:
                    self.span_artists.append(ax.axvspan(t0, t1, color=C_ESTOP, alpha=0.25,
                                                        lw=0))
                self.span_artists.append(self.axes[0].text(
                    t0, 1.0, ' e-stop', transform=self.axes[0].get_xaxis_transform(),
                    va='top', ha='left', fontsize=8, color=INK_2))
            for i, v in enumerate((status or {}).get('violations', [])):
                t0, t1 = v.get('t0'), v.get('t1')
                if t0 is None:
                    continue
                t1 = t0 + 0.2 if t1 is None or t1 <= t0 else t1
                for ax in self.axes:
                    self.span_artists.append(ax.axvspan(t0, t1, color=C_FAIL, alpha=0.15, lw=0))
                    if v.get('t') is not None:
                        self.span_artists.append(ax.axvline(v['t'], color=C_FAIL, lw=1.0))
                self.span_artists.append(self.axes[0].text(
                    t0, 0.92 - 0.07 * (i % 6), f" {v['name']}",
                    transform=self.axes[0].get_xaxis_transform(), va='top', ha='left',
                    fontsize=8, color=C_FAIL))

    return Dashboard()


def main(args=None) -> int:
    import rclpy
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
    from rclpy.utilities import remove_ros_args

    argv = sys.argv if args is None else args
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--screenshot', default='',
                    help='save the window here on exit (PNG); else the screenshot parameter')
    ap.add_argument('--exit-after', type=float, default=0.0,
                    help='close after this many wall seconds (tests)')
    opts = ap.parse_args(remove_ros_args(argv)[1:])
    if not have_display():
        print(DISPLAY_ERROR, file=sys.stderr)
        return 2

    import matplotlib
    try:
        matplotlib.use('QtAgg')
        from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
    except (ImportError, ValueError):
        matplotlib.use('Qt5Agg')
        from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
    from matplotlib.backends.qt_compat import QtCore, QtGui, QtWidgets
    from matplotlib.figure import Figure

    os.environ.pop(QT5CT_NOTE, None)
    rclpy.init(args=argv)
    model = Model()
    node = make_node(model)
    screenshot = opts.screenshot or node.screenshot
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin():
        while rclpy.ok():
            try:
                executor.spin()
            except ExternalShutdownException:
                return
            except Exception:  # noqa: B902 -- a callback bug: say so, keep receiving
                if not rclpy.ok():
                    return
                node.get_logger().error('dashboard callback failed:\n'
                                        + traceback.format_exc())

    thread = threading.Thread(target=spin, name='ros', daemon=True)
    thread.start()
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(argv[:1])
    light_palette(app, QtGui)
    win = build_window(model, QtWidgets, QtCore, QtGui, FigureCanvas, Figure)
    win.show()

    def quit_app(*_):
        app.quit()

    previous = {sig: signal.signal(sig, quit_app) for sig in (signal.SIGINT, signal.SIGTERM)}
    timer = QtCore.QTimer()
    timer.timeout.connect(win.refresh)   # also lets Python run its signal handlers
    timer.start(REDRAW_MS)
    if opts.exit_after > 0:
        QtCore.QTimer.singleShot(int(opts.exit_after * 1000), quit_app)
    code = 0
    try:
        code = app.exec_() if hasattr(app, 'exec_') else app.exec()
    finally:
        if screenshot:
            win.refresh()
            win.canvas.draw()
            app.processEvents()
            if not win.grab().save(screenshot):
                print(f'lab2_part1_dashboard: could not save {screenshot}', file=sys.stderr)
        for sig, handler in previous.items():
            signal.signal(sig, handler)
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        rclpy.try_shutdown()
        thread.join(timeout=2.0)
    return code


if __name__ == '__main__':
    sys.exit(main())
