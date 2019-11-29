export MODEL_PATH="${TF_WORKDIR_PATH}/models/${MODEL_NAME}"
tensorboard --port "${TB_PORT}" --logdir="${MODEL_PATH}"