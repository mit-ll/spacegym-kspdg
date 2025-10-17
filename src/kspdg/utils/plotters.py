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
    Env class must implement:
    - dpg_setup(history_sec) -> state
    - dpg_update(state, t, obs, *, do_draw: bool) -> None
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