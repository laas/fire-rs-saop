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


## Clone and build windninja client
RUN git clone https://github.com/firelab/windninja.git /root/windninja
RUN mkdir /root/windninja/build &&\
    cd /root/windninja/build &&\
    cmake -DNINJA_CLI=ON -DNINJAFOAM=ON -DNINJA_QTGUI=OFF .. &&\
    make -j8


##### SAOP

# install SAOP dependencies
RUN apt-get update && apt-get install -y gdb python3 cython3 python3-gdal python3-setuptools python3-affine python3-tz python3-pandas python3-matplotlib python3-sklearn python3-pip
RUN pip3 install pybind11

# install and use CLang by default
RUN apt-get install -y clang
ENV CC /usr/bin/clang
ENV CXX /usr/bin/clang++

# install "when-changed"
RUN pip3 install https://github.com/joh/when-changed/archive/master.zip

ENV WINDNINJA_CLI_PATH /root/windninja/build/src/cli

# set data directory, this directory needs to be mounted with actual data
ENV FIRERS_DATA /home/saop/data


# create a saop user
RUN useradd -ms /bin/bash saop
# Ugly hack in order to be able to call WindNinja
RUN chmod -R 777 /root
USER saop

# code directory, needs to be mounted to the the root of of the git repository
WORKDIR /home/saop/code

# For building font cache required by matplotlib
RUN python3 -c "import matplotlib.pyplot as plt"

CMD /bin/bash