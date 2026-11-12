#!/usr/bin/env python3
"""Raspberry Pi 5 timing benchmark for the hexapod control loop.

Measures (1) one policy inference and (2) whether a fixed-rate Python loop keeps
its timing while the other cores are busy, with and without OS tuning.

    python tests/bench_loop.py                 # 50 Hz, 30 s per scenario
    python tests/bench_loop.py --io-ms 3.4     # bus time measured by tests/servo_check.py
    python tests/bench_loop.py --rate 100 --load-workers 4

The tuned scenario uses the core and the real-time permission set up by scripts/setup_pi.sh.
"""
import os

# One BLAS thread: tiny matmuls only get slower and jitterier when spread over cores.
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

import argparse
import gc
import multiprocessing as mp
import platform
import time

import numpy as np


def read(path):
    try:
        with open(path) as f:
            return f.read().strip().strip("\x00")
    except OSError:
        return None


def temp_c():
    t = read("/sys/class/thermal/thermal_zone0/temp")
    return f"{int(t) / 1000:.0f} C" if t else "n/a"


def stats(ms):
    return f"p50 {np.percentile(ms, 50):6.3f}   p99 {np.percentile(ms, 99):6.3f}   max {ms.max():6.3f} ms"


def make_policy(obs_dim, hidden, act_dim):
    """Random MLP with ELU, the shape legged_gym-style policies use."""
    rng = np.random.default_rng(0)
    dims = [obs_dim, *hidden, act_dim]
    layers = [(rng.standard_normal((i, o), dtype=np.float32) / np.sqrt(i), np.zeros(o, np.float32))
              for i, o in zip(dims, dims[1:])]

    def policy(x):
        for w, b in layers[:-1]:
            x = x @ w + b
            x = np.where(x > 0, x, np.expm1(np.minimum(x, 0)))
        w, b = layers[-1]
        return x @ w + b

    return policy


def load_worker(core, stop):
    """Stands in for perception (VIO, mapping): compute bursts plus memory traffic."""
    if core is not None:
        os.sched_setaffinity(0, {core})
    a = np.random.rand(256, 256).astype(np.float32)
    big = np.zeros(8_000_000, np.float32)  # 32 MB, far bigger than the Pi 5's caches
    while not stop.is_set():
        for _ in range(5):
            a @ a
        big += 1.0


def tune(core):
    """The settings recommended for the real control process; reports which took effect."""
    notes = []
    if hasattr(os, "sched_setaffinity"):
        os.sched_setaffinity(0, {core})
        notes.append(f"pinned to core {core}")
    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(50))
        notes.append("SCHED_FIFO")
    except (AttributeError, PermissionError):
        notes.append("no SCHED_FIFO: run scripts/setup_pi.sh, then log in again")
    gc.freeze()
    gc.disable()
    notes.append("GC off")
    return notes


def control_loop(policy, obs, rate, duration, io_ms):
    period = 1.0 / rate
    n = int(rate * duration)
    late, sent = np.empty(n), np.empty(n)
    overruns = 0
    deadline = time.perf_counter() + period
    for i in range(n):
        delay = deadline - time.perf_counter()
        if delay > 0:
            time.sleep(delay)
        late[i] = time.perf_counter() - deadline
        policy(obs)
        sent[i] = time.perf_counter()  # the moment the command would go out on the bus
        # ponytail: sleep stands in for the serial sync-write + sync-read; swap in the real bus once servos are wired
        time.sleep(io_ms / 1000)
        deadline += period
        now = time.perf_counter()
        if now > deadline:  # overran: skip the missed slots instead of bursting to catch up
            overruns += 1
            deadline += np.ceil((now - deadline) / period) * period
    return late * 1000, np.abs(np.diff(sent) - period) * 1000, overruns


def scenario(name, args, policy, obs, workers, tuned):
    cores = sorted(os.sched_getaffinity(0)) if hasattr(os, "sched_getaffinity") else []
    others = [c for c in cores if c != args.loop_core]
    stop = mp.Event()
    procs = [mp.Process(target=load_worker, args=(others[i % len(others)] if tuned and others else None, stop))
             for i in range(workers)]
    for p in procs:
        p.start()
    notes = tune(args.loop_core) if tuned else []
    time.sleep(1 if procs else 0)  # let the load ramp up
    late, err, overruns = control_loop(policy, obs, args.rate, args.duration, args.io_ms)
    stop.set()
    for p in procs:
        p.join()
    print(f"\n  {name}" + (f"  [{', '.join(notes)}]" if notes else ""))
    print(f"    wake-up delay   {stats(late)}")
    print(f"    interval error  {stats(err)}")
    print(f"    overruns        {overruns} of {len(late)} cycles, CPU {temp_c()}")


def main():
    cpus = os.cpu_count() or 1
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rate", type=float, default=50, help="control rate in Hz (default 50)")
    ap.add_argument("--duration", type=float, default=30, help="seconds per scenario (default 30)")
    ap.add_argument("--io-ms", type=float, default=5, help="simulated servo bus time per cycle (default 5)")
    ap.add_argument("--obs-dim", type=int, default=250, help="63 proprioceptive + 187 height samples")
    ap.add_argument("--hidden", type=int, nargs="+", default=[512, 256, 128], help="MLP hidden layer sizes")
    ap.add_argument("--load-workers", type=int, default=max(1, cpus - 1), help="busy processes (default cores-1)")
    ap.add_argument("--loop-core", type=int, default=cpus - 1, help="core for the tuned loop (default last)")
    args = ap.parse_args()

    rt = " (PREEMPT_RT)" if read("/sys/kernel/realtime") == "1" else ""
    print(f"{read('/proc/device-tree/model') or platform.machine()}, {cpus} cores, kernel {platform.release()}{rt}, "
          f"Python {platform.python_version()}, numpy {np.__version__}")
    print(f"governor {read('/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor') or 'n/a'}, "
          f"isolated cores {read('/sys/devices/system/cpu/isolated') or 'none'}, CPU {temp_c()}")

    policy = make_policy(args.obs_dim, args.hidden, 18)
    obs = np.random.default_rng(1).standard_normal(args.obs_dim).astype(np.float32)
    for _ in range(200):  # warm-up
        policy(obs)
    t = np.empty(5000)
    for i in range(len(t)):
        t0 = time.perf_counter()
        policy(obs)
        t[i] = time.perf_counter() - t0
    print(f"\nPolicy inference, MLP {args.obs_dim}-{'-'.join(map(str, args.hidden))}-18:  {stats(t * 1000)}")

    print(f"\nControl loop at {args.rate:g} Hz ({1000 / args.rate:.1f} ms period), "
          f"{args.io_ms:g} ms simulated bus I/O, {args.duration:g} s per scenario")
    scenario("idle", args, policy, obs, 0, False)
    scenario(f"loaded, {args.load_workers} busy workers", args, policy, obs, args.load_workers, False)
    scenario(f"loaded, {args.load_workers} busy workers, tuned", args, policy, obs, args.load_workers, True)


if __name__ == "__main__":
    main()
