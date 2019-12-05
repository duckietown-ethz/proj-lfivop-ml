${WORKDIR_INIT}

${PYTHON} ${REPO_LIB_PATH}/tf_init.py

# put current date as yyyy-mm-dd HH:MM:SS in $date
printf -v date '%(%Y%m%d%H%M%S)T' -1

# set model path
export MODEL_PATH="${MODELS_WORKDIR_PATH}/${MODEL_NAME}"
echo "SET MODEL_PATH to: ${MODEL_PATH}"

# From the tensorflow/models/research/ directory
cd "${TF_PATH}/models/research"

echo "RUN inference graph export of ${MODEL_NAME} model and checkpoint ${CHECKPOINT_NUMBER}"
echo "${MODEL_PATH}/pipeline.config"
INPUT_TYPE=image_tensor
PIPELINE_CONFIG_PATH="${MODEL_PATH}/pipeline.config"
EXPORT_DIR="${EXPORT_WORKDIR_PATH}/${MODEL_NAME}_ckpt-${CHECKPOINT_NUMBER}_${date}"
TRAINED_CKPT_PREFIX="${MODEL_PATH}/model.ckpt-${CHECKPOINT_NUMBER}"

${PYTHON} object_detection/export_inference_graph.py \
    --input_type=${INPUT_TYPE} \
    --pipeline_config_path=${PIPELINE_CONFIG_PATH} \
    --trained_checkpoint_prefix=${TRAINED_CKPT_PREFIX} \
    --output_directory=${EXPORT_DIR}
