# kspdg/plot_dpg.py
import math
import time
import queue
from collections import deque
from multiprocessing import Event, Queue
import dearpygui.dearpygui as dpg

def _try_init_viewport(title: str, width=900, height=650):
    try:
        dpg.create_context()
        dpg.create_viewport(title=title, width=width, height=height)
        dpg.setup_dearpygui()
        dpg.show_viewport()
        return True, None
    except Exception as e:
        return False, e

def run_dpg_plotter(
    data_q: Queue,
    stop_evt: Event,
    env_cls,
    title: str = "kspdg – Live Telemetry",
    fps: int = 30,
    history_sec: float = 20.0,
):
    """
    ## Description
    Launches a DearPyGui window for real-time plotting.
    This function handles GUI initialization, queue polling, and frame-rate
    control, while delegating environment-specific visualization to methods
    on `env_cls`.

    ## Parameters
    - **data_q** (`multiprocessing.Queue`): Queue of `(t_seconds, obs)` tuples
    produced by the main simulation or policy loop.
    - **stop_evt** (`multiprocessing.Event`): Event used to request clean
    shutdown of the plotter process.
    - **env_cls** (`type`): Environment class that defines `dpg_setup()` and
    `dpg_update()` class methods for building and updating plots.
    - **title** (`str`): Window title shown in the viewport.
    - **fps** (`int`): Maximum GUI redraw rate (frames per second).
    - **history_sec** (`float`): Duration of the rolling time window displayed.

    ## Notes
    - Must be run in its own process; DPG requires ownership of the main thread.
    - The function blocks until the stop event is set or the window is closed.
    """
    ok, err = _try_init_viewport(title)
    if not ok:
        print("[kspdg.plot] DearPyGui init failed:", err); return

    if not callable(getattr(env_cls, "dpg_setup", None)) or \
       not callable(getattr(env_cls, "dpg_update", None)):
        print("[kspdg.plot] env_cls missing dpg_setup/dpg_update; plotting disabled.")
        dpg.destroy_context(); return

    try:
        with dpg.window(label=title, width=880, height=620):
            state = env_cls.dpg_setup(history_sec)

        target_dt = 1.0 / max(1, fps)
        last_draw = 0.0

        while dpg.is_dearpygui_running() and not stop_evt.is_set():
            # Drain queue; keep newest only
            newest_data = None
            while True:
                try: newest_data = data_q.get_nowait()
                except queue.Empty: break

            now = time.perf_counter()
            do_draw = (now - last_draw) >= target_dt
            if do_draw: last_draw = now

            if newest_data is not None:
                t, obs = newest_data
                env_cls.dpg_update(state, t, obs, do_draw=do_draw)
            elif do_draw:
                # Optional: allow the env to redraw even without new data (moving axes, etc.)
                env_cls.dpg_update(state, None, None, do_draw=True)

            dpg.render_dearpygui_frame()
    except KeyboardInterrupt:
        pass
    finally:
        dpg.destroy_context()