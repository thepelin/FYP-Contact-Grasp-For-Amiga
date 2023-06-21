ifneq ("$(wildcard .env)","")
include .env
export $(shell sed 's/=.*//' .env)
current_dir = $(shell pwd)
else
$(shell cp .env.example .env)
$(warning No .env found; I have created one. PLEASE UPDATE ITS VALUES (vim .env))
exit 1
endif

# Used to handle multi-user simultaneous usage
DOCKERID = $(shell pwd | cut -d "/" -f4)



####################################################################################################
#################################### BUILDING THE DOCKER IMAGES ####################################
####################################################################################################


######################################## ORIN ARM WITH ZED #########################################
_create_ros_dl_cache:
	@mkdir -p .calibrated_models/

build-armcamera: _create_ros_dl_cache
	docker build -t personalroboticsimperial/prl:amiga_arm_camera -f dockerfiles/Dockerfile.armcamera .

build-armcamera-full: _create_ros_dl_cache
	docker build -t personalroboticsimperial/prl:amiga_arm_camera_deps -f dockerfiles/deps/Dockerfile.armcamera-deps .
	docker build -t personalroboticsimperial/prl:amiga_arm_camera -f dockerfiles/Dockerfile.armcamera .


############################################## LAPTOP ##############################################
build-external-mid:
	docker build -t personalroboticsimperial/prl:amiga_arm_external_mid -f dockerfiles/Dockerfile.external-mid .
	docker build -t personalroboticsimperial/prl:amiga_arm_external_${DOCKERID} -f dockerfiles/Dockerfile.external .

build-obj-det:
	docker build -t personalroboticsimperial/prl:sim_obj_det -f dockerfiles/Dockerfile.sim_obj_det .
	
build-contact-graspnet:
	docker build -t personalroboticsimperial/prl:contact_graspnet -f dockerfiles/Dockerfile.contact_graspnet .

build-external:
	docker build -t personalroboticsimperial/prl:amiga_arm_external_${DOCKERID} -f dockerfiles/Dockerfile.external .

build-external-full:
	docker build -t personalroboticsimperial/prl:amiga_arm_external_deps -f dockerfiles/deps/Dockerfile.external-deps .
	docker build -t personalroboticsimperial/prl:amiga_arm_external_${DOCKERID} -f dockerfiles/Dockerfile.external .

build-calib:
	docker build -t personalroboticsimperial/prl:amiga_arm_hec -f dockerfiles/Dockerfile.handeyecalib .

rebuild-ik:	
	docker exec amiga_ext_${DOCKERID} bash -c "source devel/setup.bash && catkin build collision_ik_moveit_wrapper prl_ur_kinematics ur_kinematics"

rebuild-rust-ik:	
	docker exec amiga_ext_${DOCKERID} bash -c "source devel/setup.bash && source ../.cargo/env && cd src/ur10e_robotiq/relaxed_ik/src/RelaxedIK_Rust && cargo build --bin relaxed_ik_cmd"



####################################################################################################
###################################### RUNNNING THE CONTAINERS #####################################
####################################################################################################
_start_container_if_not_running:
	@if [ ! $$(docker ps -a | grep amiga_ext_${DOCKERID}) ]; then $(MAKE) run-external; fi

stop:
	@docker stop amiga_ext_${DOCKERID} > /dev/null 2>&1 || true && docker rm amiga_ext_${DOCKERID} > /dev/null 2>&1 || true
	@docker stop am_collisionIK > /dev/null 2>&1 || true && docker rm am_collisionIK > /dev/null 2>&1 || true
	@docker stop amiga_arm > /dev/null 2>&1 || true && docker rm amiga_arm > /dev/null 2>&1 || true

description: _start_container_if_not_running
	@docker exec -it amiga_ext_${DOCKERID} bash -c "source /root/ros_ws/devel/setup.bash; roslaunch amiga_description full.launch --wait"

arm-driver: _start_container_if_not_running
	@docker exec -it amiga_ext_${DOCKERID} bash -c "source /root/ros_ws/devel/setup.bash; roslaunch amiga_driver startup.launch --wait"

arm-moveit: _start_container_if_not_running init-gripper
	@docker exec -it amiga_ext_${DOCKERID} bash -c "source /root/ros_ws/devel/setup.bash && \
		rosservice call /ur_hardware_interface/dashboard/power_on && \
		sleep 20 && \

		rosservice call /ur_hardware_interface/resend_robot_program && \
		sleep 1 && \
		roslaunch amiga_driver startup_moveit.launch --wait"

arm-moveit-simple: _start_container_if_not_running
	@docker exec -it amiga_ext_${DOCKERID} bash -c "source /root/ros_ws/devel/setup.bash && \
		rosservice call /ur_hardware_interface/resend_robot_program && \
		sleep 1 && \
		roslaunch amiga_driver startup_moveit.launch --wait"

arm-moveit-gazebo: _start_container_if_not_running
	@docker exec -it amiga_ext_${DOCKERID} bash -c "source /root/ros_ws/devel/setup.bash && roslaunch amiga_driver startup_moveit.launch --wait"

arm-off: _start_container_if_not_running
	@docker exec amiga_ext_${DOCKERID} bash -c "source devel/setup.bash && rosservice call /ur_hardware_interface/dashboard/power_off"

arm-on: _start_container_if_not_running init-gripper
	@docker exec -it amiga_ext_${DOCKERID} bash -c "source /root/ros_ws/devel/setup.bash && \
		rosservice call /ur_hardware_interface/dashboard/power_on && \
		sleep 18 && \
		rosservice call /ur_hardware_interface/dashboard/unlock_protective_stop && \
		sleep 5 && \
		rosservice call /ur_hardware_interface/dashboard/brake_release && \
		sleep 2 && \
		rosservice call /ur_hardware_interface/resend_robot_program"

arm-reset: _start_container_if_not_running
	@docker exec -it amiga_ext_${DOCKERID} bash -c "source /root/ros_ws/devel/setup.bash && \
		rosservice call /ur_hardware_interface/dashboard/unlock_protective_stop && \
		sleep 2 && \
		rosservice call /ur_hardware_interface/resend_robot_program && \
		sleep 2 && \
		rosservice call /ur_hardware_interface/resend_robot_program"

gazebo: _start_container_if_not_running
	@docker exec amiga_ext_${DOCKERID} bash -c "source devel/setup.bash && roslaunch amiga_sim gazebo_basic.launch"

rviz: _start_container_if_not_running
	@docker exec amiga_ext_${DOCKERID} bash -c "source devel/setup.bash && roslaunch amiga_driver rviz.launch"

init-gripper:
	@docker exec amiga_ext_${DOCKERID} bash -c "source devel/setup.bash && rosservice call /amiga_gripper/init_gripper"
	@docker exec amiga_ext_${DOCKERID} bash -c "source devel/setup.bash && rosservice call /amiga_gripper/close_gripper"

open-gripper:
	@docker exec amiga_ext_${DOCKERID} bash -c "source devel/setup.bash && rosservice call /amiga_gripper/open_gripper"

close-gripper:
	@docker exec amiga_ext_${DOCKERID} bash -c "source devel/setup.bash && rosservice call /amiga_gripper/close_gripper"

showcase-gripper:
	@docker exec amiga_ext_${DOCKERID} bash -c "source devel/setup.bash && \
		rosservice call /amiga_gripper/open_gripper && sleep 5 && \
		rosservice call /amiga_gripper/basic_mode_gripper && sleep 2 && \
		rosservice call /amiga_gripper/close_gripper && sleep 4 && \
		rosservice call /amiga_gripper/scissor_mode_gripper && sleep 7 && \
		rosservice call /amiga_gripper/pinch_mode_gripper && sleep 4 && \
		rosservice call /amiga_gripper/wide_mode_gripper && sleep 5 && \
		rosservice call /amiga_gripper/basic_mode_gripper && sleep 5 && \
		rosservice call /amiga_gripper/open_gripper"

manip-server:
	@docker exec -it amiga_ext_${DOCKERID} bash -c "source devel/setup.bash && roslaunch amiga_moveit_wrapper wrapper_v2.launch"

cook:
	@docker exec -it amiga_ext_${DOCKERID} bash -c "source devel/setup.bash && roslaunch amiga_cooking_assistant cooking_assistant.launch"

record_zed:
	@docker exec -it amiga_ext_${DOCKERID} bash -c "source devel/setup.bash && rosbag record /zed2 _node/rgb/image_rect_color -O src/ur10e_robotiq/rosbags/`date +'%Y%m%d_%H%M%S.bag'`"

_run: 
	xhost +local:root || true
	xhost +SI:localuser:$(whoami) || true
	docker run \
		--rm \
		--detach \
		-e "DISPLAY" \
		-e "QT_X11_NO_MITSHM=1" \
		-e "XAUTHORITY=${XAUTH}" \
		-e ROS_IP=$(AMIGA_ROS_IP) \
		-e ROS_MASTER_URI="http://${AMIGA_ROSCORE_IP}:${AMIGA_ROSCORE_PORT}" \
		-e AMIGA_ROSCORE_PORT=${AMIGA_ROSCORE_PORT} \
		-e AMIGA_ENABLE_ARM=${AMIGA_ENABLE_ARM} \
		-e AMIGA_RUN_SLAM_ON=${AMIGA_RUN_SLAM_ON} \
		-e AMIGA_SLAM_MODE=${AMIGA_SLAM_MODE} \
		-e AMIGA_ODOM=${AMIGA_ODOM} \
		-e AMIGA_RTABMAP_ARNIE_LOC_DB_PATH=${AMIGA_RTABMAP_ARNIE_LOC_DB_PATH} \
		-e AMIGA_JOYSTICK=${AMIGA_JOYSTICK} \
		-e AMIGA_ENABLE_MOVE_BASE=${AMIGA_ENABLE_MOVE_BASE} \
		-e AMIGA_SIMULATION=${AMIGA_SIMULATION} \
		-e RNET_ENABLE_MOVE_BASE=${AMIGA_ENABLE_MOVE_BASE} \
		-e RNET_SLAM_MODE=${AMIGA_SLAM_MODE} \
		-e RNET_RUN_SLAM_ON=${AMIGA_RUN_SLAM_ON} \
		-e RNET_JOYSTICK=${AMIGA_JOYSTICK} \
		-e RNET_RTABMAP_ARNIE_LOC_DB_PATH=${AMIGA_RTABMAP_ARNIE_LOC_DB_PATH} \
		-e RNET_ODOM=${AMIGA_ODOM} \
		-v $(current_dir)/amiga_driver/cfg/zed_calib/SN21894833.conf:/usr/local/zed/settings/SN21894833.conf \
		-v $(current_dir)/amiga_driver/cfg/zed_resources:/usr/local/zed/resources/ \
		-v $(current_dir)/.calibrated_models/SSD-Mobilenet-v2/:/usr/local/bin/networks/SSD-Mobilenet-v2/ \
		-v $(current_dir):/root/ros_ws/src/ur10e_robotiq/ \
		-v $(current_dir)/amiga_collisionIK/start_here.py/:/root/ros_ws/src/ur10e_robotiq/relaxed_ik/src/start_here.py \
		-v $(current_dir)/amiga_collisionIK/config/:/root/ros_ws/src/ur10e_robotiq/relaxed_ik/src/RelaxedIK/Config/ \
		-v $(current_dir)/amiga_collisionIK/config/urdfs/:/root/ros_ws/src/ur10e_robotiq/relaxed_ik/src/RelaxedIK/urdfs/ \
		-v $(current_dir)/amiga_sim/gazebo_cache/:/root/.gazebo/  \
		-v ~/.Xauthority:/root/.Xauthority:rw \
		-v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
		-v /tmp/.docker.xauth:/tmp/.docker.xauth \
		--privileged \
		--network host \
		--name $(CONTAINER_NAME) \
		--gpus all \
		--runtime nvidia \
		-t \
		$(TARGET_IMAGE)

run-external: TARGET_IMAGE="personalroboticsimperial/prl:amiga_arm_external_${DOCKERID}"
run-external: AMIGA_ROS_IP=${AMIGA_COMPUTER_IP}
run-external: CONTAINER_NAME="amiga_ext_${DOCKERID}"
run-external: _run

run-obj-det: TARGET_IMAGE="personalroboticsimperial/prl:sim_obj_det"
run-obj-det: AMIGA_ROS_IP=${AMIGA_OBJ_IP}
run-obj-det: CONTAINER_NAME="sim_obj_det"
run-obj-det: _run

stop-obj:
	@docker stop sim_obj_det

run-contact-graspnet: TARGET_IMAGE="personalroboticsimperial/prl:contact_graspnet"
run-contact-graspnet: AMIGA_ROS_IP=${AMIGA_CG_IP}
run-contact-graspnet: CONTAINER_NAME="contact_graspnet"
run-contact-graspnet: _run

stop-contact-graspnet:
	@docker stop contact_graspnet

run-armcamera: TARGET_IMAGE="personalroboticsimperial/prl:amiga_arm_camera"
run-armcamera: AMIGA_ROS_IP=${AMIGA_JETSON_ARM_IP}
run-armcamera: CONTAINER_NAME="amiga_arm"
run-armcamera: _run

restart-navigation:
	ssh ${AMIGA_PI_USER}@${AMIGA_PI_IP} "cd rnet-wheelchair-docker && make run-pi"
	scp .env ${AMIGA_JETSON_BASE_USER}@${AMIGA_JETSON_BASE_IP}:/home/${AMIGA_JETSON_BASE_USER}/rnet-wheelchair-docker/.env
	ssh -X ${AMIGA_JETSON_BASE_USER}@${AMIGA_JETSON_BASE_IP} "cd rnet-wheelchair-docker && sed -i 's=AMIGA_=RNET_=' .env && make run-jetson"

restart-obj-detect:
	scp .env ${AMIGA_JETSON_ARM_USER}@${AMIGA_JETSON_ARM_IP}:/home/${AMIGA_JETSON_ARM_USER}/src/amiga_main/.env
	ssh -X ${AMIGA_JETSON_ARM_USER}@${AMIGA_JETSON_ARM_IP} "cd src/amiga_main/ && make run-armcamera"

stop-navigation:
	ssh -X ${AMIGA_JETSON_BASE_USER}@${AMIGA_JETSON_BASE_IP} "docker container stop rnet_base"

stop-obj-detect:
	ssh -X ${AMIGA_JETSON_ARM_USER}@${AMIGA_JETSON_ARM_IP} "docker container stop amiga_arm"


####################################################################################################
############################################ DOCKER HUB ############################################
####################################################################################################

push-jetson-arm:
	docker push personalroboticsimperial/prl:amiga_arm_camera

push-external:
	docker push personalroboticsimperial/prl:amiga_arm_external 


####################################################################################################
######################################## ENTERING CONTAINERS #######################################
####################################################################################################

exec:
	docker exec -it amiga_ext_${DOCKERID} /bin/bash


exec-obj:
	docker exec -it sim_obj_det /bin/bash

exec-contact-graspnet:
	docker exec -it contact_graspnet /bin/bash

exec-arm:
	docker exec -it amiga_arm /bin/bash

