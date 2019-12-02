export MODEL_PATH="${WORKDIR_PATH}/models/${MODEL_NAME}"
tensorboard --port "${TB_PORT}" --logdir="${MODEL_PATH}"