# kspdg/plot_dpg.py
import math
import time
import queue
from collections import deque
from multiprocessing import Event, Queue
import dearpygui.dearpygui as dpg

def _try_init_viewport(title: str, width=900, height=500):
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
    title: str = "kspdg – Live Plot",
    fps: int = 30,
    history_sec: float = 10.0,
    ingest_cap_hz: int = 240,
    show_sine_if_no_data: bool = True,
):
    """
    ## Description
    Runs a standalone DearPyGui process for lightweight real-time plotting.
    Intended to be launched as a `multiprocessing.Process` alongside the main
    simulation or environment runner.

    ## Parameters
    - data_q (`multiprocessing.Queue`):  
    Queue providing `(t_seconds, value)` tuples to plot. Typically small
    (e.g., `maxsize=1`) so old samples are dropped when new ones arrive.

    - stop_evt (`multiprocessing.Event`):  
    Event used to signal shutdown. When set, the plot window closes cleanly.

    - title (`str`):  
    Window title (default: `"kspdg – Live Plot"`).

    - fps (`int`):  
    Max redraw rate in frames per second.

    - history_sec (`float`):  
    Duration (in seconds) of the rolling time window displayed on the x-axis.

    - ingest_cap_hz (`int`):  
    Approximate max input rate used to size the ring buffer.

    - show_sine_if_no_data (`bool`):  
    If `True`, displays a sine wave until real data arrives.

    ## Notes
    - Designed for use in a separate process; DearPyGui must own the main thread
    of its process.
    - Safe to use even if an OpenGL context can't be created—prints a warning
    and exits gracefully.
    - The plotting loop is self-contained and non-blocking relative to the main app.
    """

    ok, err = _try_init_viewport(title)
    if not ok:
        print("[kspdg.plot] DearPyGui couldn't create an OpenGL context/viewport.")
        print(f"Reason: {err}")
        print("Tip: update graphics drivers or continue without plotting.")
        return

    # Ring buffers
    max_points = int(history_sec * ingest_cap_hz)
    xs = deque(maxlen=max_points)
    ys = deque(maxlen=max_points)

    with dpg.window(label=title, width=880, height=460):
        with dpg.plot(label="live", height=420, width=860):
            x_axis = dpg.add_plot_axis(dpg.mvXAxis, label="time (s, last window)")
            y_axis = dpg.add_plot_axis(dpg.mvYAxis, label="value")
            series_tag = dpg.add_line_series([], [], parent=y_axis, label="signal")

    dpg.set_axis_limits(y_axis, -1.2, 1.2)  # safe default; adjust live if you want

    target_dt = 1.0 / max(1, fps)
    last_draw = 0.0
    t0 = time.perf_counter()

    try:
        while dpg.is_dearpygui_running() and not stop_evt.is_set():
            now = time.perf_counter()

            # Drain queue quickly; keep only newest
            newest = None
            while True:
                try:
                    newest = data_q.get_nowait()
                except queue.Empty:
                    break

            if newest is not None:
                t, y = newest
                xs.append(float(t))
                ys.append(float(y))
            elif show_sine_if_no_data:
                # Synth data so you can see the pipeline before hooking obs
                t = now - t0
                y = math.sin(2.0 * math.pi * 0.5 * t)
                xs.append(t)
                ys.append(y)

            # Draw at capped FPS
            if (now - last_draw) >= target_dt and xs:
                last_draw = now
                t_latest = xs[-1]
                tmin = t_latest - history_sec

                # Compute start index (simple scan; small buffers so OK)
                # Convert to list once to avoid repeated deque indexing
                x_list = list(xs)
                y_list = list(ys)
                start = 0
                for i in range(len(x_list) - 1, -1, -1):
                    if x_list[i] < tmin:
                        start = i + 1
                        break
                x_vis = x_list[start:]
                y_vis = y_list[start:]

                # Relative time to keep x-axis in [-history, 0]
                x_rel = [xi - t_latest for xi in x_vis]
                dpg.set_value(series_tag, [x_rel, y_vis])

                # Optional cheap autoscale for y (every N frames)
                # y_min, y_max = min(y_vis), max(y_vis)
                # pad = 0.05 * max(1e-6, (y_max - y_min))
                # dpg.set_axis_limits(y_axis, y_min - pad, y_max + pad)

                dpg.set_axis_limits(x_axis, -history_sec, 0.0)

            dpg.render_dearpygui_frame()

    except KeyboardInterrupt:
        pass
    finally:
        dpg.destroy_context()
