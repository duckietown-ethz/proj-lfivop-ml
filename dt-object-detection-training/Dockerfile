# Options:

# Set --build-arg TF_PACKAGE_VERSION=1.11.0rc0 to install a specific version.
# Installs the version 1.15.0 by default.
ARG TF_PACKAGE_VERSION="1.15.0"

# Set --build-arg GPU=-gpu to use gpu
# For the 1.15 release, CPU and GPU support are included in a single package
ARG GPU=""

# we use python 3
FROM tensorflow/tensorflow:${TF_PACKAGE_VERSION}${GPU}-py3 as base

# See http://bugs.python.org/issue19846
ENV LANG C.UTF-8
ENV TZ=Europe/Zurich

# we use python 3
ARG _PY_SUFFIX=3
ENV PYTHON=python${_PY_SUFFIX}
ARG PIP=pip${_PY_SUFFIX}

# prevent interactive questions
ARG DEBIAN_FRONTEND=noninteractive

# install tensorboard
RUN ${PIP} install "tensorboard"${TF_PACKAGE_VERSION:+==${TF_PACKAGE_VERSION}}

ARG REPO_NAME="dt-object-detection-training"
ENV REPO_PATH="/src/${REPO_NAME}"

ENV MODEL_NAME="ssdlite_mobilenet_v2_coco"

# create repo directory
RUN mkdir -p "${REPO_PATH}"

WORKDIR "${REPO_PATH}"

# copy requirements files
COPY ./requirements-apt.txt "${REPO_PATH}/"
COPY ./requirements-py.txt "${REPO_PATH}/"

# install apt dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
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

ENV TF_WORKDIR_PATH="/tf_workdir"

# define launch scripts
ENV TRAINING_LAUNCHFILE "${REPO_PATH}/launch/training_launch.sh"
RUN chmod a+rwx ${TRAINING_LAUNCHFILE}
ENV TENSORBOARD_LAUNCHFILE "${REPO_PATH}/launch/tensorboard_launch.sh"
RUN chmod a+rwx ${TENSORBOARD_LAUNCHFILE}

# port for tensorboard
ENV TB_PORT 6006

# define command
# CMD ["bash", "-c", "source /etc/bash.bashrc &&  && jupyter notebook --notebook-dir=${TF_PATH} --ip 0.0.0.0 --no-browser --allow-root"]
CMD ["bash", "-c", "source /etc/bash.bashrc && ${TRAINING_LAUNCHFILE}"]

# maintainer
LABEL maintainer="Maximilian Stoelzle (maximilian@stoelzle.ch)"