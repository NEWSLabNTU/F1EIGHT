#!/usr/bin/env python3
"""Sample what a workload costs this machine, for sizing one to run AutoSDV.

Answers the question a person picking a laptop actually has -- how much memory,
how many cores, how much GPU -- rather than what any one process did. So the
numbers are system-wide deltas against an idle baseline taken at startup, which
is what a machine with less of everything would have to find.

    scripts/profiling/host_resource_sampler.py -o tmp/build.csv -- just build
    scripts/profiling/host_resource_sampler.py -o tmp/sim.csv --seconds 240

The first form samples until the command exits and reports its wall time; the
second samples for a fixed period while something else runs. Both print a
summary: peak memory over baseline, peak swap, CPU utilisation, GPU memory.

CPU utilisation is reported two ways, because both matter when sizing: the
percentage of *this* machine, and the same figure as whole cores, which is what
transfers to a machine with a different core count.
"""
import argparse
import csv
import os
import re
import shutil
import subprocess
import sys
import time


def _descendants(root):
    """PIDs of every process below `root`, by parent links."""
    children = {}
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        try:
            with open(f"/proc/{pid}/stat") as fh:
                ppid = int(fh.read().rsplit(") ", 1)[1].split()[1])
        except (FileNotFoundError, ProcessLookupError, PermissionError, IndexError):
            continue
        children.setdefault(ppid, []).append(int(pid))
    found, todo = set(), [root]
    while todo:
        for child in children.get(todo.pop(), ()):
            if child not in found:
                found.add(child)
                todo.append(child)
    return found


def match_processes(pattern, pgid=None):
    """RSS sum and CPU jiffies of the workload's processes.

    Selected either by a regex over the command line, or -- more reliably for a
    launched stack -- by process group: play_launch puts every node and
    container in its own group, and matching on names misses the ones whose
    binary is named after neither the launcher nor the container.

    Tracking the workload itself, rather than the machine, is what makes a
    number usable on a box that is also doing something else -- which is the
    normal case, and was the case when these were measured.
    """
    rss = 0
    jiffies = 0
    count = 0
    tree = _descendants(pgid) if pgid is not None else None
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        try:
            with open(f"/proc/{pid}/stat") as fh:
                stat = fh.read()
            fields_all = stat.rsplit(") ", 1)[1].split()
            if pgid is not None:
                # the group leader's own group, plus everything it spawned: play_launch
                # puts each node in a group of its own, so the group alone is three processes
                if int(fields_all[2]) != pgid and int(pid) not in tree:
                    continue
            else:
                with open(f"/proc/{pid}/cmdline", "rb") as fh:
                    cmdline = fh.read().replace(b"\0", b" ").decode("utf-8", "replace")
                if not pattern.search(cmdline):
                    continue
            with open(f"/proc/{pid}/statm") as fh:
                rss += int(fh.read().split()[1]) * os.sysconf("SC_PAGE_SIZE")
            jiffies += int(fields_all[11]) + int(fields_all[12])   # utime + stime
            count += 1
        except (FileNotFoundError, ProcessLookupError, PermissionError, IndexError):
            continue
    return rss, jiffies, count


def read_meminfo():
    info = {}
    with open("/proc/meminfo") as fh:
        for line in fh:
            key, _, rest = line.partition(":")
            info[key] = int(rest.split()[0]) * 1024  # kB -> bytes
    used = info["MemTotal"] - info["MemAvailable"]
    swap_used = info["SwapTotal"] - info["SwapFree"]
    return used, swap_used, info["MemTotal"]


def read_cpu():
    with open("/proc/stat") as fh:
        parts = fh.readline().split()[1:]
    values = [int(v) for v in parts]
    idle = values[3] + values[4]          # idle + iowait
    return sum(values), idle


def read_gpu():
    if not shutil.which("nvidia-smi"):
        return None, None
    try:
        out = subprocess.run(
            ["nvidia-smi", "--query-gpu=memory.used,utilization.gpu",
             "--format=csv,noheader,nounits"],
            capture_output=True, text=True, timeout=5).stdout.strip()
    except Exception:
        return None, None
    mem = util = 0
    for line in out.splitlines():          # sum over all GPUs
        m, u = (f.strip() for f in line.split(","))
        mem += int(m) * 1024 * 1024
        util = max(util, int(u))
    return mem, util


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-o", "--out", default="tmp/resources.csv")
    ap.add_argument("--interval", type=float, default=2.0)
    ap.add_argument("--seconds", type=float, default=0.0,
                    help="sample for this long; default is until the command exits")
    ap.add_argument("--label", default="")
    ap.add_argument("--pgid", type=int, default=0,
                    help="track this process group instead of a name regex; a launched "
                         "stack is one group, so nothing is missed and nothing else counted")
    ap.add_argument("--match", default="",
                    help="regex over cmdlines; reports those processes' own RSS and CPU "
                         "as well as the machine's, so a busy neighbour does not inflate it")
    ap.add_argument("command", nargs=argparse.REMAINDER,
                    help="after --, the command to run and measure")
    args = ap.parse_args()

    cmd = args.command[1:] if args.command[:1] == ["--"] else args.command
    ncpu = len(
        [ln for ln in open("/proc/stat") if ln.startswith("cpu") and ln[3].isdigit()])

    base_mem, base_swap, total_mem = read_meminfo()
    prev_total, prev_idle = read_cpu()
    base_gpu, _ = read_gpu()
    base_gpu = base_gpu or 0

    pattern = re.compile(args.match) if args.match else None
    pgid = args.pgid or None
    if pgid is not None:
        pattern = pattern or re.compile(".")   # unused, but enables the block below
    hz = os.sysconf("SC_CLK_TCK")
    prev_jiffies = None

    proc = subprocess.Popen(cmd) if cmd else None
    start = time.time()
    rows = []
    peak = {"mem": 0, "swap": 0, "cpu": 0.0, "gpu_mem": 0, "gpu_util": 0,
            "match_rss": 0, "match_cores": 0.0, "match_count": 0}

    try:
        while True:
            time.sleep(args.interval)
            used, swap, _ = read_meminfo()
            cur_total, cur_idle = read_cpu()
            dt, didle = cur_total - prev_total, cur_idle - prev_idle
            prev_total, prev_idle = cur_total, cur_idle
            cpu_pct = 100.0 * (1 - didle / dt) if dt else 0.0
            gpu_mem, gpu_util = read_gpu()
            gpu_mem = gpu_mem or 0

            row = {
                "t": round(time.time() - start, 1),
                "mem_used_bytes": used,
                "mem_over_baseline_bytes": max(0, used - base_mem),
                "swap_used_bytes": swap,
                "cpu_percent": round(cpu_pct, 1),
                "cpu_cores": round(cpu_pct * ncpu / 100.0, 2),
                "gpu_mem_bytes": gpu_mem,
                "gpu_util_percent": gpu_util or 0,
            }
            if pattern is not None:
                rss, jiffies, count = match_processes(pattern, pgid)
                cores = 0.0
                if prev_jiffies is not None and args.interval > 0:
                    cores = max(0.0, (jiffies - prev_jiffies) / hz / args.interval)
                prev_jiffies = jiffies
                row["match_rss_bytes"] = rss
                row["match_cores"] = round(cores, 2)
                row["match_procs"] = count
                peak["match_rss"] = max(peak["match_rss"], rss)
                peak["match_cores"] = max(peak["match_cores"], cores)
                peak["match_count"] = max(peak["match_count"], count)

            rows.append(row)
            peak["mem"] = max(peak["mem"], row["mem_over_baseline_bytes"])
            peak["swap"] = max(peak["swap"], max(0, swap - base_swap))
            peak["cpu"] = max(peak["cpu"], cpu_pct)
            peak["gpu_mem"] = max(peak["gpu_mem"], max(0, gpu_mem - base_gpu))
            peak["gpu_util"] = max(peak["gpu_util"], row["gpu_util_percent"])

            if proc is not None and proc.poll() is not None:
                break
            if args.seconds and (time.time() - start) >= args.seconds:
                break
    except KeyboardInterrupt:
        pass

    elapsed = time.time() - start
    rc = proc.returncode if proc is not None else 0

    if rows:
        with open(args.out, "w", newline="") as fh:
            writer = csv.DictWriter(fh, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)

    gb = 1024 ** 3
    mean_cores = sum(r["cpu_cores"] for r in rows) / len(rows) if rows else 0
    print(f"\n== {args.label or ' '.join(cmd) or 'sample'} ==")
    print(f"  wall time         {elapsed:8.0f} s")
    print(f"  peak memory       {peak['mem']/gb:8.1f} GiB over a {base_mem/gb:.1f} GiB baseline"
          f"  (machine has {total_mem/gb:.0f} GiB)")
    print(f"  peak swap         {peak['swap']/gb:8.1f} GiB")
    print(f"  peak CPU          {peak['cpu']:8.1f} % of {ncpu} cores"
          f"  = {peak['cpu']*ncpu/100:.1f} cores")
    print(f"  mean CPU                   {mean_cores:.1f} cores")
    print(f"  peak GPU memory   {peak['gpu_mem']/gb:8.1f} GiB")
    print(f"  peak GPU util     {peak['gpu_util']:8d} %")
    if pattern is not None:
        label = f"process group {pgid}" if pgid else f"processes matching {args.match!r}"
        print(f"  -- {label} --")
        print(f"  peak own memory   {peak['match_rss']/gb:8.1f} GiB")
        print(f"  peak own CPU      {peak['match_cores']:8.1f} cores")
        print(f"  peak process count{peak['match_count']:8d}")
    print(f"  samples           {len(rows):8d} -> {args.out}")
    return rc


if __name__ == "__main__":
    sys.exit(main())
