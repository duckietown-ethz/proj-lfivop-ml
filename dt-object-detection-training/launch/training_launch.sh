${PYTHON} ${REPO_PATH}/tf_test.py

# set model path
export MODEL_PATH="${TF_WORKDIR_PATH}/models/${MODEL_NAME}"
echo "SET MODEL_PATH to: ${MODEL_PATH}"
# create model directories
mkdir -p "${MODEL_PATH}/eval"
mkdir -p "${MODEL_PATH}/train"

# From the tensorflow/models/research/ directory
cd "${TF_PATH}/models/research"

echo "RUN TF ModelBuilder test"
${PYTHON} object_detection/builders/model_builder_test.py

# set TensorFlow logging settings
# 0 = all messages are logged (default behavior)
# 1 = INFO messages are not printed
# 2 = INFO and WARNING messages are not printed
# 3 = INFO, WARNING, and ERROR messages are not printed
export TF_CPP_MIN_LOG_LEVEL=2

echo "RUN TF training with the ${MODEL_NAME} model"
PIPELINE_CONFIG_PATH="${MODEL_PATH}/pipeline.config"
NUM_TRAIN_STEPS=50000
SAMPLE_1_OF_N_EVAL_EXAMPLES=1
python object_detection/model_main.py \
    --pipeline_config_path="${PIPELINE_CONFIG_PATH}" \
    --model_dir="${MODEL_PATH}" \
    --num_train_steps=${NUM_TRAIN_STEPS} \
    --sample_1_of_n_eval_examples=$SAMPLE_1_OF_N_EVAL_EXAMPLES \
    --alsologtostderr