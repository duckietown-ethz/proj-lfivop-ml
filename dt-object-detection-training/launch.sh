export TF_OBJDET_PATH="/models/research/object_detection"

${PYTHON} ${REPO_PATH}/tf_test.py

# From the tensorflow/models/research/ directory
cd "${TF_PATH}/models/research"

echo "RUN TF ModelBuilder test"
${PYTHON} object_detection/builders/model_builder_test.py


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