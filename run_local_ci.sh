rosrun industrial_ci run_ci ROS_DISTRO=melodic ROS_REPO=main DOCKER_RUN_OPTS="-e EMSDK -e EM_CONFIG -e EM_CACHE -e EMSDK_NODE -v ${EMSDK}:${EMSDK}"
