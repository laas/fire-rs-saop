FROM abitmonn/windninja-cli:latest


##### SAOP

# install SAOP dependencies
RUN apt-get update && apt-get install -y gdb python3 cython3 python3-gdal python3-setuptools python3-affine\
 python3-tz python3-pandas python3-matplotlib python3-pip ffmpeg clang libboost-all-dev

RUN pip3 install pybind11==2.2.1
RUN pip3 install joblib

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
    && chown -R saop:saop $HOME
USER saop

ENV WINDNINJA_CLI_PATH /usr/local/bin

# set data directory, this directory needs to be mounted with actual data
ENV FIRERS_DATA /home/saop/data

ENV PYTHONPATH ${PYTHONPATH}:/home/saop/code/python

# code directory, needs to be mounted to the the root of of the git repository
WORKDIR /home/saop/code

# For building font cache required by matplotlib
RUN python3 -c "import matplotlib.pyplot as plt"

CMD /bin/bash
