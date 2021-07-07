FROM ubuntu:18.04
RUN apt-get update && apt-get install -y apt-utils

# Windninja
RUN apt-get install -y build-essential cmake git sudo wget libproj-dev libfontconfig1-dev\
    libcurl4-gnutls-dev libnetcdf-dev libboost-program-options-dev libboost-date-time-dev\
    libgeos-dev libboost-test-dev libgdal-dev

## Setup windninja environment
ENV WINDNINJA_CLI_PREFIX /usr/local
WORKDIR /root/windninja

## Clone and build windninja client
RUN git clone --depth 1 https://github.com/firelab/windninja.git /root/windninja/windninja
RUN mkdir -p /root/windninja/windninja/build &&\
    cd /root/windninja/windninja/build &&\
    cmake -DNINJA_CLI=ON -DNINJAFOAM=ON -DNINJA_QTGUI=OFF .. &&\
    make -j8
## install to /usr/local/bin
RUN cd /root/windninja/windninja/build && make install
## Clean up
RUN rm -rf /root/windninja

# SAOP
## Disable tzdata interactive configuration
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
## install SAOP dependencies
RUN apt-get install -y g++ gdb cmake python3 cython3 python3-gdal python3-setuptools\
 python3-pip python3-affine python3-tz python3-pandas python3-numpy python3-matplotlib ffmpeg\
 clang libboost-all-dev libgdal-dev python3-opencv

RUN pip3 install pybind11
RUN pip3 install joblib
RUN pip3 install scikit-image
RUN pip3 install scipy
RUN pip3 install six

# Clean Apt cache
RUN apt-get clean && \
    rm -rf /var/lib/apt/lists/* && \
    rm -rf /tmp/*

ARG USER_UID
ARG USER_GROUP

ENV HOME /home/saop
RUN groupadd ${USER_GROUP:+-g} ${USER_GROUP} saop \
    && useradd ${USER_UID:+-u} ${USER_UID} -g saop -ms /bin/bash saop \
    && mkdir -p $HOME \
    && mkdir -p $HOME/code \
    && chown -R saop:saop $HOME
USER saop

ENV WINDNINJA_CLI_PATH ${WINDNINJA_CLI_PREFIX}/bin
ENV PYTHONPATH ${PYTHONPATH}:/home/saop/code/python

# code directory, needs to be mounted to the the root of of the git repository
WORKDIR /home/saop

# For building font cache required by matplotlib
RUN python3 -c "import matplotlib.pyplot as plt"

# Clone fire-rs-data
RUN git clone https://github.com/laas/fire-rs-data.git data

# Set data directory to a known dataset
ENV FIRERS_DATA /home/saop/data/porto_utm/25m

# Clone fire-rs-saop and init pybind11 submodule
RUN git clone https://github.com/laas/fire-rs-saop.git code
WORKDIR /home/saop/code
RUN git submodule init && git submodule update

CMD /bin/bash
