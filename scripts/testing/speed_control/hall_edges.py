#!/usr/bin/env python3
"""Passive hall sensor check with blocking edge waits (no polling, drives nothing).

Usage: hall_edges.py SECONDS [BOARD pin, default 15]   (see README.md)
Waits on BOTH edges with GPIO.wait_for_edge, timestamps each, then reports
rising/falling counts and HIGH/LOW pulse widths.
"""
import sys, time
import Jetson.GPIO as GPIO

secs = float(sys.argv[1])
pin = int(sys.argv[2]) if len(sys.argv) > 2 else 15
GPIO.setmode(GPIO.BOARD)
GPIO.setup(pin, GPIO.IN)
edges = []  # (t, level after edge)
t_end = time.monotonic() + secs
print(f'pin {pin}: waiting for edges for {secs:.0f} s, start level {GPIO.input(pin)}', flush=True)
try:
    while True:
        left_ms = int((t_end - time.monotonic()) * 1000)
        if left_ms <= 0:
            break
        if GPIO.wait_for_edge(pin, GPIO.BOTH, timeout=left_ms) is None:
            break
        t = time.monotonic()
        edges.append((t, GPIO.input(pin)))
finally:
    GPIO.cleanup()

rising = sum(1 for _, lv in edges if lv == 1)
falling = len(edges) - rising
print(f'edges={len(edges)} rising={rising} falling={falling}')
high = [b[0] - a[0] for a, b in zip(edges, edges[1:]) if a[1] == 1]
low = [b[0] - a[0] for a, b in zip(edges, edges[1:]) if a[1] == 0]
for name, w in (('HIGH', high), ('LOW', low)):
    if w:
        w = sorted(w)
        print(f'{name} widths: n={len(w)} min={w[0]*1e3:.2f} ms median={w[len(w)//2]*1e3:.2f} ms max={w[-1]*1e3:.1f} ms')
if len(edges) > 1:
    span = edges[-1][0] - edges[0][0]
    print(f'first..last edge: {span:.1f} s, rising rate {rising / span:.1f}/s' if span > 0 else '')
