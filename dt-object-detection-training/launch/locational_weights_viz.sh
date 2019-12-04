${WORKDIR_INIT}

${PYTHON} ${REPO_LIB_PATH}/tf_init.py

${PYTHON} ${REPO_LIB_PATH}/dt_locational_weights/dt_locational_weights_viz.py "$@"
