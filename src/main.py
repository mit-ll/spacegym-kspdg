import krpc
import time
from src.ksp_env.ksp_env import GameEnv
import math
import numpy as np

conn = krpc.connect(name="Tracker")
# env = GameEnv(conn)
# vessel = env.vessel

# frame = vessel.orbit.body.reference_frame
# vert_speed = conn.add_stream(getattr, vessel.flight(frame), "vertical_speed")

# while True:
#     print(vert_speed)
#     time.sleep(1 / 10)
