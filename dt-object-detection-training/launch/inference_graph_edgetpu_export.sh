# Attention:
# this script requires a quantized trained model with 300x300 pixel images

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
PIPELINE_CONFIG_PATH="${MODEL_PATH}/pipeline.config"
EXPORT_DIR="${EXPORT_WORKDIR_PATH}/${MODEL_NAME}_ckpt-${CHECKPOINT_NUMBER}_${date}"
TRAINED_CKPT_PREFIX="${MODEL_PATH}/model.ckpt-${CHECKPOINT_NUMBER}"

${PYTHON} object_detection/export_tflite_ssd_graph.py \
    --pipeline_config_path="${PIPELINE_CONFIG_PATH}" \
    --trained_checkpoint_prefix="${TRAINED_CKPT_PREFIX}" \
    --output_directory="${EXPORT_DIR}"

tflite_convert \
  --graph_def_file="${EXPORT_DIR}/tflite_graph.pb" \
  --output_file="${EXPORT_DIR}/tflite_graph.tflite" \
  --input_array="normalized_input_image_tensor" \
  --output_array='TFLite_Detection_PostProcess','TFLite_Detection_PostProcess:1','TFLite_Detection_PostProcess:2','TFLite_Detection_PostProcess:3' \
  --input_shape=1,320,320,3 \
  --inference_type=QUANTIZED_UINT8 \
  --mean_values=128 \
  --std_dev_values=127 \
  --allow_custom_ops \
  --change_concat_input_ranges=false \
  --allow_nudging_weights_to_use_fast_gemm_kernel=true

edgetpu_compiler "${EXPORT_DIR}/tflite_graph.tflite" \
  --out_dir "${EXPORT_DIR}"
