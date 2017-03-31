FROM abitmonn/windninja-cli:latest

RUN apt-get update && apt-get install -y python3 cython3 python3-gdal python3-setuptools python3-affine python3-tz python3-pandas python3-matplotlib python3-sklearn



ENV WINDNINJA_CLI_PATH /root/windninja/build/src/cli
ENV FIRERS_DATA /home/saop/data

RUN useradd -ms /bin/bash saop
USER saop
WORKDIR /home/saop/code


CMD /bin/bash