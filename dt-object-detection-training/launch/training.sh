${WORKDIR_INIT}

${PYTHON} ${REPO_LIB_PATH}/tf_init.py

# generate locational weights for safety-modified loss function
${PYTHON} ${REPO_LIB_PATH}/dt_locational_weights/dt_locational_weights_gen.py

# set model path
export MODEL_PATH="${MODELS_WORKDIR_PATH}/${MODEL_NAME}"
echo "SET MODEL_PATH to: ${MODEL_PATH}"

# create model directories
# mkdir -p "${MODEL_PATH}/eval"
# mkdir -p "${MODEL_PATH}/train"

# From the tensorflow/models/research/ directory
cd "${TF_PATH}/models/research"

echo "RUN TF ModelBuilder test"
${PYTHON} object_detection/builders/model_builder_test.py

echo "RUN TF training with the ${MODEL_NAME} model"
PIPELINE_CONFIG_PATH="${MODEL_PATH}/pipeline.config"
SAMPLE_1_OF_N_EVAL_EXAMPLES=1
${PYTHON} object_detection/model_main.py \
    --pipeline_config_path="${PIPELINE_CONFIG_PATH}" \
    --model_dir="${MODEL_PATH}" \
    --num_train_steps=${NUM_TRAIN_STEPS} \
    --sample_1_of_n_eval_examples=$SAMPLE_1_OF_N_EVAL_EXAMPLES \
    --alsologtostderr