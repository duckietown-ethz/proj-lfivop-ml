# Copyright 2018 The TensorFlow Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ============================================================================
#
# THIS IS A GENERATED DOCKERFILE.
#
# This file was assembled from multiple pieces, whose use is documented
# throughout. Please refer to the TensorFlow dockerfiles documentation
# for more information.

ARG UBUNTU_VERSION=18.04

FROM ubuntu:${UBUNTU_VERSION} as base

ARG USE_PYTHON_3_NOT_2
# ARG _PY_SUFFIX=${USE_PYTHON_3_NOT_2:+3}
ARG _PY_SUFFIX=3
ENV PYTHON=python${_PY_SUFFIX}
ARG PIP=pip${_PY_SUFFIX}

# prevent interactive questions
ARG DEBIAN_FRONTEND=noninteractive

# See http://bugs.python.org/issue19846
ENV LANG C.UTF-8
ENV TZ=Europe/Zurich

RUN apt-get update && apt-get install -y \
    ${PYTHON} \
    ${PYTHON}-pip

RUN ${PIP} --no-cache-dir install --upgrade \
    pip \
    setuptools

# Some TF tools expect a "python" binary
RUN ln -s $(which ${PYTHON}) /usr/local/bin/python

# Options:
#   tensorflow
#   tensorflow-gpu
#   tf-nightly
#   tf-nightly-gpu
# Set --build-arg TF_PACKAGE_VERSION=1.11.0rc0 to install a specific version.
# Installs the latest version by default.
ARG TF_PACKAGE=tensorflow
ARG TF_PACKAGE_VERSION="1.15.0"
RUN ${PIP} install ${TF_PACKAGE}${TF_PACKAGE_VERSION:+==${TF_PACKAGE_VERSION}}
RUN ${PIP} install "tensorboard"${TF_PACKAGE_VERSION:+==${TF_PACKAGE_VERSION}}

COPY bashrc /etc/bash.bashrc
RUN chmod a+rwx /etc/bash.bashrc

ARG REPO_NAME="dt-object-detection-training"
ENV REPO_PATH="/src/${REPO_NAME}"

ARG MODEL_NAME="ssdlite_mobilenet_v2_coco"

# create repo directory
RUN mkdir -p "${REPO_PATH}"

WORKDIR "${REPO_PATH}"

# copy requirements files
COPY ./requirements-apt.txt "${REPO_PATH}/"
COPY ./requirements-py.txt "${REPO_PATH}/"

# install apt dependencies
RUN apt-get install -y --no-install-recommends \
    $(awk -F: '/^[^#]/ { print $1 }' "${REPO_PATH}/"requirements-apt.txt | uniq) \
  && rm -rf /var/lib/apt/lists/*

# install python dependencies
RUN ${PIP} install -r ${REPO_PATH}/requirements-py.txt

RUN jupyter serverextension enable --py jupyter_http_over_ws

# configure tensorflow
ENV TF_PATH="${REPO_PATH}/tf"
ENV TF_MODELS_RESEARCH_PATH="${TF_PATH}/models/research"
WORKDIR ${TF_PATH}

# Install tensorflow models object detection
RUN git clone https://github.com/tensorflow/models ${TF_PATH}/models
# WORKDIR ${TF_PATH}/models
# RUN git checkout "tags/v1.13.0"

# install COCO API
RUN git clone https://github.com/cocodataset/cocoapi.git /tmp/cocoapi
WORKDIR /tmp/cocoapi/PythonAPI
RUN make
RUN make install
RUN ${PYTHON} setup.py install
RUN cp -r pycocotools ${TF_MODELS_RESEARCH_PATH}

# From tensorflow/models/research/
WORKDIR ${TF_MODELS_RESEARCH_PATH}
RUN protoc object_detection/protos/*.proto --python_out=.
RUN export PYTHONPATH="$PYTHONPATH:${TF_MODELS_RESEARCH_PATH}:${TF_MODELS_RESEARCH_PATH}/slim:${TF_MODELS_RESEARCH_PATH}/object_detection"
RUN ${PYTHON} setup.py build
RUN ${PYTHON} setup.py install
WORKDIR ${TF_MODELS_RESEARCH_PATH}/slim
RUN ${PYTHON} setup.py install

RUN apt-get autoremove -y

WORKDIR ${TF_PATH}
EXPOSE 8888

RUN ${PYTHON} -m ipykernel.kernelspec

# install tutorials
# RUN mkdir -p ${TF_PATH}/tensorflow-tutorials && chmod -R a+rwx ${TF_PATH}/
# RUN mkdir /.local && chmod a+rwx /.local
# WORKDIR ${TF_PATH}/tensorflow-tutorials
# RUN wget https://raw.githubusercontent.com/tensorflow/docs/master/site/en/tutorials/keras/classification.ipynb
# RUN wget https://raw.githubusercontent.com/tensorflow/docs/master/site/en/tutorials/keras/overfit_and_underfit.ipynb
# RUN wget https://raw.githubusercontent.com/tensorflow/docs/master/site/en/tutorials/keras/regression.ipynb
# RUN wget https://raw.githubusercontent.com/tensorflow/docs/master/site/en/tutorials/keras/save_and_load.ipynb
# RUN wget https://raw.githubusercontent.com/tensorflow/docs/master/site/en/tutorials/keras/text_classification.ipynb
# RUN wget https://raw.githubusercontent.com/tensorflow/docs/master/site/en/tutorials/keras/text_classification_with_hub.ipynb
# RUN wget https://raw.githubusercontent.com/tensorflow/models/master/research/object_detection/object_detection_tutorial.ipynb


WORKDIR ${REPO_PATH}
# copy the source code
COPY . "${REPO_PATH}/"

# create repo directories
RUN mkdir -p "${REPO_PATH}/data"
ENV MODEL_PATH="${REPO_PATH}/models/${MODEL_NAME}"
RUN echo "SET MODEL_PATH to: ${MODEL_PATH}"
RUN mkdir -p "${MODEL_PATH}/eval"
RUN mkdir -p "${MODEL_PATH}/train"

# define launch script
ENV LAUNCHFILE "${REPO_PATH}/launch.sh"
RUN chmod a+rwx ${LAUNCHFILE}

# define command
# CMD ["bash", "-c", "source /etc/bash.bashrc &&  && jupyter notebook --notebook-dir=${TF_PATH} --ip 0.0.0.0 --no-browser --allow-root"]
CMD ["bash", "-c", "source /etc/bash.bashrc && ${LAUNCHFILE}"]

# maintainer
LABEL maintainer="Maximilian Stoelzle (maximilian@stoelzle.ch)"