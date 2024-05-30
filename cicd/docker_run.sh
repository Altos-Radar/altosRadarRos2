cd ./docker
sudo docker build -t altos_docker:v2.0 .
cd ..

xhost +local:root;

sudo docker run -it --privileged --net=host \
    -v /home:/home \
    -v /dev/bus/usb:/dev/bus/usb  \
    -v /media:/media \
    --gpus all \
    --device=/dev/video2 \
    --group-add video \
    --volume=/tmp/.X11-unix:/tmp/.X11-unix \
    --env="DISPLAY=$DISPLAY" \
    --privileged=true \
    --name=altos_ros_humble_env \
    altos_docker:v2.0 /bin/bash
