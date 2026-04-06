# ph
fusion RFID BRISC


git clone https://github.com/therobocademy/ros2_nvidia_isaac_bootcamp.git



git clone https://github.com/therobocademy/ros2_nvidia_isaac_bootcamp.git

cd ros2_nvidia_isaac_bootcamp

xhost +local:docker

export DISPLAY=${DISPLAY:-:0}
export DOCKER_NETWORK_MODE=host

mkdir -p ~/docker/isaac-sim/cache/{kit,ov,pip,glcache,computecache} ~/docker/isaac-sim/{logs,data,documents}

docker compose up workshop



therobocademy/ros2_nvidia_workshop:latest

docker exec -it isaac-sim bash


chmod -R 777 ~/docker/isaac-sim

./scripts/launch_isaac_sim.sh
