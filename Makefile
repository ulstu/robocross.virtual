-MAKEFLAGS+=--silent
UID:=$(shell id +-u)
DOCKER_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
#PROJECT_DIR:=$(shell dirname ${DOCKER_DIR})
PROJECT_DIR:=${DOCKER_DIR}
NVIDIA_GPU:=$(shell (docker info | grep Runtimes | grep nvidia 1> /dev/null && command -v nvidia-smi 1>/dev/null 2>/dev/null && nvidia-smi | grep Processes 1>/dev/null 2>/dev/null) && echo '--runtime nvidia --gpus all' || echo '')
IMAGE=ulstu
FLAVOR ?= devel
ROBOT_PANEL_PORT ?= 8008
VS_PORT ?= 31415
WEBOTS_STREAM_PORT ?= 1234
DISPLAY=:1

.PHONY: all

#

all: run copy-working-files disable-ros2-dds start-code-server 
colors:
	$(eval NC=\033[1;0m)
	$(eval RED=\033[1;31m)
	$(eval GREEN=\033[1;32m)
	$(eval BOLD=\033[1;37m)

build:
	echo ${NO_CACHE_ARG}
	DOCKER_BUILDKIT=1 docker build ${DOCKER_DIR} -f ${DOCKER_DIR}/Dockerfile.base -t ulstu ${DOCKER_ARGS} --build-arg UID=${UID}

buildmacos:
	echo ${NO_CACHE_ARG}
	docker build --platform=linux/amd64 ${DOCKER_DIR} -f ${DOCKER_DIR}/Dockerfile.base -t ulstu ${DOCKER_ARGS} --build-arg UID=${UID} 
#--runtime=nvidia 

run: 
	docker run --ipc=host \
		--cap-add SYS_ADMIN \
		--name ${IMAGE}-${FLAVOR} \
		--add-host=host.docker.internal:host-gateway \
		--privileged \
		--restart unless-stopped \
		-p ${ROBOT_PANEL_PORT}:8008 -p ${VS_PORT}:31415 -p ${WEBOTS_STREAM_PORT}:1234 \
		-e NVIDIA_DRIVER_CAPABILITIES=all ${NVIDIA_GPU} \
		-e DISPLAY=${DISPLAY} \
		-v ~/.Xauthority:/ulstu/.host/.Xauthority:ro \
		-e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
      	-e DISPLAY=:1 \
		--device=/dev/dxg \
		-it --gpus all --device /dev/dri:/dev/dri \
		-e LD_LIBRARY_PATH=/usr/lib/wsl/lib \
		-v /tmp/.X11-unix/:/tmp/.X11-unix:rw \
		-v /dev:/dev:rw \
		-v ${PROJECT_DIR}/projects/devel:/ulstu/repositories:rw \
		-d -it ${IMAGE} 1>/dev/null

#--net=host 
#-v /usr/lib/wsl:/usr/lib/wsl 
#-v /mnt/wslg:/mnt/wslg 

test-nvidia: colors
	lspci | grep -qi nvidia && base64 --decode massage | unxz || true
	docker run --rm \
		-e NVIDIA_DRIVER_CAPABILITIES=all ${NVIDIA_GPU} \
		-e DISPLAY=${DISPLAY} \
		${IMAGE} | grep -qi 'nvidia' && \
	printf '%b\n' "${RED}Detected NVIDIA GPU in system, but missing packets, look up NVIDIA GPU section in README!\n${NC}" || \
	true

copy-working-files:
	docker exec -it ${IMAGE}-${FLAVOR} ln -s /ulstu/repositories/webots_ros2_suv /ulstu/ros2_ws/src/webots_ros2_suv && \
	docker exec -it ${IMAGE}-${FLAVOR} ln -s /ulstu/repositories/robot_interfaces /ulstu/ros2_ws/src/robot_interfaces ; \

disable-ros2-dds:
	@RANDOM_ID=$$(shuf -i 1-255 -n 1); \
	BASHRC=/ulstu/.bashrc; \
	if docker exec ${IMAGE}-${FLAVOR} sh -c "grep -q 'export ROS_DOMAIN_ID=' $$BASHRC"; then \
		docker exec ${IMAGE}-${FLAVOR} sh -c "sed -i 's/export ROS_DOMAIN_ID=.*/export ROS_DOMAIN_ID=$$RANDOM_ID/' $$BASHRC"; \
	else \
		docker exec ${IMAGE}-${FLAVOR} sh -c "echo 'export ROS_DOMAIN_ID=$$RANDOM_ID' >> $$BASHRC"; \
	fi

copy-working-folders:
	if [ -d ${PROJECT_DIR}/docker/projects/${FLAVOR}/]; then \
        echo "project dir exists. Remove it first" ; \
	else \
        mkdir ${PROJECT_DIR}/docker/projects/${FLAVOR} && \
        cp -r ${PROJECT_DIR}/projects/devel/webots_ros2_suv ${PROJECT_DIR}/docker/projects/${FLAVOR}/webots_ros2_suv  && \
        cp -r ${PROJECT_DIR}/projects/devel/robot_interfaces ${PROJECT_DIR}/docker/projects/${FLAVOR}/robot_interfaces ; \
    fi


td:
	echo ${PROJECT_DIR}

start-code-server:
	docker exec -d -it ulstu-${FLAVOR} bash -c 'pgrep code-server || code-server /ulstu/ros2_ws' && \
	xdg-open 'localhost:31415?folder=/ulstu/ros2_ws'

stop-code-server:
	docker exec -it ulstu-${FLAVOR} pkill -f code-server

exec:
	docker exec -it ulstu-${FLAVOR} bash

destroy:
	docker container kill ulstu-${FLAVOR} 1>/dev/null || true
	docker container rm -f ulstu-${FLAVOR} 1>/dev/null || true

setup-default: colors
	docker exec -it ulstu-${FLAVOR} sh -c '/usr/bin/setup.sh --first-time-ros-setup'

setup-interactive: colors
	docker exec -it ulstu-${FLAVOR} sh -c '/usr/bin/setup.sh --interactive'
	printf '%b\n%b\n' "${GREEN}Interactive setup complete!${NC}" \
		"Run ${BOLD}make exec${NC} or ${BOLD}docker exec -it ulstu-${FLAVOR}${NC} to access the container"

