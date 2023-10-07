# Use a base image with apt package manager
FROM adrianorenstein/pytorch:2.0.1-python3.10.13-devell

# swig for https://github.com/Farama-Foundation/Gymnasium/blob/main/docs/environments/box2d.md
RUN pip3 install --user --no-cache-dir \
    gymnasium[classic-control] \
    swig gymnasium[box2d] \
    krpc \
    poliastro


# RUN git clone https://github.com/rusty1s/pytorch_scatter.git ~/pytorch_scatter && pip install --user --no-cache-dir -e ~/pytorch_scatter

# RUN pip3 install --user --no-cache-dir redis backpack-for-pytorch wandb moviepy imageio \
#     && wandb login --relogin "APIKEYHERE"

# install self as module
COPY setup.py /app/setup.py
RUN pip3 install --user --no-cache-dir -e /app

CMD [ "/bin/bash" ]