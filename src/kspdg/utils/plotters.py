# kspdg/plot_dpg.py
import time
import queue
from multiprocessing import Event, Queue
import dearpygui.dearpygui as dpg

def _try_init_viewport(title: str, width=1280, height=720):
    try:
        dpg.create_context()
        dpg.create_viewport(title=title, width=width, height=height)
        dpg.setup_dearpygui()
        dpg.show_viewport()
        return True, None
    except Exception as e:
        return False, e

def run_dpg_telem_plotter(
    data_q: Queue,
    stop_evt: Event,
    env_cls,
    title: str = "kspdg – Live Telemetry",
    fps: int = 30,
    history_sec: float = 20.0,
):
    """
    ## Description
    Launches a DearPyGui window for real-time telemetry plotting.
    This function handles GUI initialization, queue polling, and frame-rate
    control, while delegating environment-specific visualization to methods
    on `env_cls`.

    ## Parameters
    - **data_q** (`multiprocessing.Queue`): Queue of `(t_seconds, obs)` tuples
    produced by the main simulation or policy loop.
    - **stop_evt** (`multiprocessing.Event`): Event used to request clean
    shutdown of the plotter process.
    - **env_cls** (`type`): Environment class that defines `dpg_telem_setup()` and
    `dpg_telem_update()` class methods for building and updating plots.
    - **title** (`str`): Window title shown in the viewport.
    - **fps** (`int`): Maximum GUI redraw rate (frames per second).
    - **history_sec** (`float`): Duration of the rolling time window displayed.

    ## Notes
    - Must be run in its own process; DPG requires ownership of the main thread.
    - The function blocks until the stop event is set or the window is closed.
    """
    ok, err = _try_init_viewport(title)
    if not ok:
        print("[kspdg.plotter] DearPyGui init failed:", err); return

    if not callable(getattr(env_cls, "dpg_telem_setup", None)) or \
       not callable(getattr(env_cls, "dpg_telem_update", None)):
        print("[kspdg.plotter] env_cls missing dpg_telem_setup/dpg_telem_update; plotting disabled.")
        dpg.destroy_context(); return

    try:
        with dpg.window(label=title,  width=1280, height=720):
            state = env_cls.dpg_telem_setup(history_sec)

        target_dt = 1.0 / max(1, fps)
        last_draw = 0.0

        # Keep rendering telem window frames until user manually closes window
        while dpg.is_dearpygui_running():
            now = time.perf_counter()
            do_draw = (now - last_draw) >= target_dt
            if do_draw:
                last_draw = now

            newest_data = None

            # Only consume data while we're not "stopped"
            if not stop_evt.is_set():
                # Drain queue; keep newest only
                while True:
                    try:
                        newest_data = data_q.get_nowait()
                    except queue.Empty:
                        break

            if newest_data is not None:
                t, obs = newest_data
                env_cls.dpg_telem_update(state, t, obs, do_draw=do_draw)
            elif do_draw:
                # Optional: redraw even without new data (static view)
                env_cls.dpg_telem_update(state, None, None, do_draw=True)

            dpg.render_dearpygui_frame()

    except KeyboardInterrupt:
        pass
    finally:
        dpg.destroy_context()