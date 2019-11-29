${PYTHON} ${REPO_SRC_PATH}/tf_init.py

mkdir -p "${TF_WORKDIR_PATH}/data"

${PYTHON} ${REPO_SRC_PATH}/dt_dataset_preparation.py