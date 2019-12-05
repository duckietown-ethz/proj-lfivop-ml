# parameters
ARG REPO_NAME="coral"

# ==================================================>
# ==> Do not change this code
ARG ARCH=arm32v7
ARG MAJOR=daffy
ARG BASE_TAG=${MAJOR}-${ARCH}
ARG BASE_IMAGE=dt-ros-commons

# define base image
FROM duckietown/${BASE_IMAGE}:${BASE_TAG}

# define repository path
ARG REPO_NAME
ARG REPO_PATH="${CATKIN_WS_DIR}/src/${REPO_NAME}"
WORKDIR "${REPO_PATH}"

# create repo directory
RUN mkdir -p "${REPO_PATH}"

# copy dependencies files only
COPY ./dependencies-apt.txt "${REPO_PATH}/"
COPY ./dependencies-py.txt "${REPO_PATH}/"

# Python 3 requirement
#RUN curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
#RUN python3.5 get-pip.py

RUN apt-get update
RUN apt-get install -y python3-pip
RUN pip3 install -r ${REPO_PATH}/dependencies-py.txt

# install apt dependencies
RUN apt-get update \
  && apt-get install -y --no-install-recommends \
    $(awk -F: '/^[^#]/ { print $1 }' dependencies-apt.txt | uniq) \
  && rm -rf /var/lib/apt/lists/*

# Add GLIBCXX 3.4.22 for TFLite
RUN add-apt-repository -y ppa:ubuntu-toolchain-r/test
RUN apt-get update
RUN apt-get install -y gcc-4.9
RUN apt-get upgrade -y libstdc++6

# Install Edge TPU Runtime
RUN echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" | sudo tee /etc/apt/sources.list.d/coral-edgetpu.list
RUN curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key add -
RUN apt-get update
RUN apt-get install -y libedgetpu1-std
RUN apt-get install -y edgetpu-examples

# Install the TensorFlow Lite library
RUN wget https://dl.google.com/coral/python/tflite_runtime-1.14.0-cp35-cp35m-linux_armv7l.whl
RUN pip3 install tflite_runtime-1.14.0-cp35-cp35m-linux_armv7l.whl

RUN mkdir coral && cd coral
RUN git clone https://github.com/google-coral/tflite.git
#RUN cd tflite/python/examples/classification
#RUN bash install_requirements.sh

# install python dependencies
# RUN pip install -r ${REPO_PATH}/dependencies-py.txt

# copy the source code
COPY . "${REPO_PATH}/"

# build packages
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
  catkin build \
    --workspace ${CATKIN_WS_DIR}/

# define launch script
ENV LAUNCHFILE "${REPO_PATH}/launch.sh"

# Object Detection Safety Emergency Stop
# emergency stop is activated, if duckie or duckiebot are right in front of Duckiebot
ENV OBJDET_EMERGENCY_STOP=true

# define command
CMD ["bash", "-c", "${LAUNCHFILE}"]
# <== Do not change this code
# <==================================================

# maintainer
LABEL maintainer="<YOUR_FULL_NAME> (<YOUR_EMAIL_ADDRESS>)"
