FROM ubuntu:16.04

##### Windninja

RUN apt-get update && apt-get install -y cmake git build-essential sudo wget libfontconfig1-dev libcurl4-gnutls-dev libnetcdf-dev  libboost-program-options-dev libboost-date-time-dev libgeos-dev libboost-test-dev
# RUN apt-get install -y qt4-dev-tools libqtwebkit-dev  # only needed for windninja gui

ENV PREFIX /usr/local
ENV POPPLER "poppler-0.23.4"
ENV PROJ "proj-4.8.0"
ENV GDAL "gdal-2.0.3"

RUN mkdir /root/src
WORKDIR /root/src

## Get and build poppler for PDF support in GDAL
RUN wget http://poppler.freedesktop.org/$POPPLER.tar.xz &&\
    tar -xvf $POPPLER.tar.xz 
WORKDIR $POPPLER/
RUN ./configure --prefix=$PREFIX --enable-xpdf-headers &&\
    make install -j8 &&\
    make clean


## Get and build proj
WORKDIR /root/src
RUN wget http://download.osgeo.org/proj/$PROJ.tar.gz &&\
    tar xvfz $PROJ.tar.gz
WORKDIR $PROJ
RUN ./configure --prefix=$PREFIX &&\
    make install -j8 &&\
    make clean
RUN cp $PREFIX/include/proj_api.h $PREFIX/lib


## Get and build GDAL with poppler support
WORKDIR /root/src
RUN wget http://download.osgeo.org/gdal/2.0.3/$GDAL.tar.gz &&\
    tar -xvf $GDAL.tar.gz 
WORKDIR $GDAL/
RUN ./configure --prefix=$PREFIX --with-poppler=$PREFIX &&\
    make -j8 &&\
    make install &&\
    make clean


# create a saop user
RUN useradd -ms /bin/bash saop
USER saop
WORKDIR /home/saop

## Clone and build windninja client
RUN git clone --depth 1 https://github.com/firelab/windninja.git /home/saop/windninja
RUN mkdir /home/saop/windninja/build &&\
    cd /home/saop/windninja/build &&\
    cmake -DNINJA_CLI=ON -DNINJAFOAM=ON -DNINJA_QTGUI=OFF .. &&\
    make -j8


USER root
# install to /usr/local/bin
RUN cd /home/saop/windninja/build && make install

# Remove windninja build dir
RUN rm -rf /home/saop/windninja


# Set root password to "root"
RUN echo "root:root" | chpasswd


##### SAOP

# install SAOP dependencies
RUN apt-get update && apt-get install -y gdb python3 cython3 python3-gdal python3-setuptools python3-affine\
 python3-tz python3-pandas python3-matplotlib python3-pip ffmpeg clang libboost-test-dev
RUN pip3 install pybind11==2.2.1
run pip3 install joblib
# install "when-changed"
RUN pip3 install https://github.com/joh/when-changed/archive/master.zip

# Clean Apt cache
RUN apt-get clean && \
    rm -rf /var/lib/apt/lists/* && \
    rm -rf /tmp/*

USER saop

# use CLang by default
# ENV CC /usr/bin/clang
# ENV CXX /usr/bin/clang++


ENV WINDNINJA_CLI_PATH /usr/local/bin

# set data directory, this directory needs to be mounted with actual data
ENV FIRERS_DATA /home/saop/data

# code directory, needs to be mounted to the the root of of the git repository
WORKDIR /home/saop/code

# For building font cache required by matplotlib
RUN python3 -c "import matplotlib.pyplot as plt"

CMD /bin/bash