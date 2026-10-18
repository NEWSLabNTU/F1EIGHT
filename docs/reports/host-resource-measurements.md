# What AutoSDV costs a host machine, measured

The figures behind the book's "What Machine You Need" page. They live here
rather than there because a student choosing a laptop needs one table and a
yes/no about the GPU; the methodology, the per-workload breakdown and the
reasons a number is what it is are ours to maintain, not theirs to read.

**Measured on**: Ubuntu 22.04, Intel Core Ultra 7 270K Plus (24 cores), 125 GB
RAM, RTX 5090, with `scripts/profiling/host_resource_sampler.py`. Memory is the
machine's own usage over an idle baseline, which is what a smaller machine has
to find. CPU is in whole cores, the figure that transfers to a machine with a
different core count.

## Per workload

| Workload | Peak RAM | CPU at startup | CPU in steady state |
|---|---|---|---|
| `just build`, clean workspace | **11.3 GiB** | every core available | — |
| Planning simulation | **2.5 GiB** | 19 cores, briefly | ~2 cores |
| Logging simulation, CPU path | **3.4 GiB** | 21 cores, briefly | ~3 cores |
| Logging simulation, GPU path | 3.4 GiB + ~1 GiB VRAM | same | ~3 cores |
| RC simulator (`just sim rc`), no RViz, pinned to 4 CPUs | **2.6 GiB** | not captured | ~1.6 cores |
| RViz, added to any of them | ~1.4 GiB | — | ~1 core |

The RC simulator is planning-simulator mode with the real vehicle interface and
the rc_sim plant (pure Python, 1 kHz) in place of simple_planning_simulator;
measured with `scripts/testing/rc_sim/smoke.py --taskset 0-3 --resources`.
Figures are the stack's own processes (56 of them, memory as RSS, CPU as whole
cores), sampled from localization initialization through the engage, the drive
and the stop, so the startup burst is not captured (the quietest run's 1.95-core
maximum is a steady-state maximum). On a quiet host (load average 12 to 21 on 32
cores) three runs of the fixed stack (Task 18b) used a median 1.6 cores steady
(maximum 1.9) and 2.5 GiB peak; Task 20's quietest run measured 1.8 cores and the
plant 0.19 core (spec: under 0.2). Two passing runs of that task while neighbours
held the load at 22 to 37 measured the same 2.6 GiB but 3.0 to 3.4 cores and 0.30
core for the plant: CPU time inflates when four pinned CPUs are shared, so the
budget of 3 cores is met on a quiet machine and only reached, not beaten, on a
busy one. The plant was not re-measured in Task 18b. The smoke test's earlier
unreliability (the car creeping at about 0.15 m/s in one of three unmodified runs
and both `clock_period` 0.01 runs) was the MRM flap, not load or `clock_period`:
see the emergency flap in the rc-sim guide. After the fix `smoke.py --taskset 0-3`
passed three of three at load 12 to 21. No lever was applied.

**The build peak is the largest, and it is adjustable.** 11.3 GiB is colcon
compiling 31 packages with 24 jobs in parallel; the trace spends about twelve
seconds there and then falls away. The peak scales with how many compilers run
at once, so a 4-core laptop reaches roughly a quarter of it without being asked.
On a memory-short machine, cap it rather than discovering the OOM killer:

```bash
colcon build --base-paths src --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 2
```

**"CPU at startup" is a burst, not a requirement.** Launching a stack starts
thirty-odd nodes at once and they use every core present -- 21 of 24 here. On
four cores the same work takes longer and nothing fails. What a machine must
sustain is the steady-state column: two to three cores.

**The build is quick because most of Autoware is already built.** 31 packages in
**89 seconds** here. AutoSDV is a workspace *over* a binary Autoware install, so
`just build` compiles the vehicle's own packages and not the 30 GB underneath.
Minutes, not hours -- and on four cores, still minutes.

## Disk

| | |
|---|---|
| Autoware Debian, downloaded | 1.9 GB |
| Autoware, installed at `/opt/autoware/1.5.0` | 4.8 GB |
| This repository, cloned (includes the COSS map) | ~1 GB |
| Workspace after `just build` (`build/` + `install/`) | 0.9 GB |
| COSS rosbag, downloaded | 1.6 GB |
| COSS rosbag, unpacked | 2.8 GB |
| **Total, everything, after cleanup** | **~11 GB** |

20 GB free is the floor because the rosbag's download and its unpacked copy
coexist for a while, and because `play_log/` grows with every run. 40 GB to
record your own bags, the one item here with no natural size.

## The GPU

Neither simulation requires one.

- The planning simulation has no sensors, no perception and no localization. It
  never touches a GPU.
- The logging simulation with `pose_source:=ndt launch_perception:=false` is
  pure CPU and holds the sensor's full rate -- measured 10.06 Hz against a 10 Hz
  recording.

What a GPU buys: `pose_source:=cuda_ndt` (~1 GiB VRAM) and perception's TensorRT
models (VRAM, plus the engine build -- see
`docs/roadmap/11-engine-file-delivery.md`).

**RViz wants a real graphics stack.** Over a plain VNC server with software
rendering it drew **1 fps** on this machine: enough to confirm something is on
screen, useless for watching a vehicle drive.

## Re-measuring

```bash
scripts/profiling/host_resource_sampler.py -o tmp/host.csv
```

`scripts/profiling/README.md` says what each column means. Re-run after an
Autoware upgrade or a change to what `just build` compiles; the numbers above
are one machine on one day, not a contract.
