export TF_OBJDET_PATH="/models/research/object_detection"

${PYTHON} ${REPO_PATH}/tf_test.py

# From the tensorflow/models/research/ directory
cd "${TF_PATH}/models/research"
MODEL_DIR="${REPO_PATH}/models/dt-object-detection-model"

${PYTHON} object_detection/builders/model_builder_test.py

PIPELINE_CONFIG_PATH="${MODEL_DIR}/ssdlite_mobilenet_v2_coco.config"
NUM_TRAIN_STEPS=50000
SAMPLE_1_OF_N_EVAL_EXAMPLES=1
#python object_detection/model_main.py \
#    --pipeline_config_path=${PIPELINE_CONFIG_PATH} \
#    --model_dir=${MODEL_DIR} \
#    --num_train_steps=${NUM_TRAIN_STEPS} \
#    --sample_1_of_n_eval_examples=$SAMPLE_1_OF_N_EVAL_EXAMPLES \
#    --alsologtostderr