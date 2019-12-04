${WORKDIR_INIT_PATH}

export MODEL_PATH="${MODELS_WORKDIR_PATH}/${MODEL_NAME}"
tensorboard --port "${TB_PORT}" --logdir="${MODEL_PATH}"