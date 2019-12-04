${WORKDIR_INIT}

export MODEL_PATH="${MODELS_WORKDIR_PATH}/${MODEL_NAME}"
tensorboard --port "${TB_PORT}" --logdir="${MODEL_PATH}"