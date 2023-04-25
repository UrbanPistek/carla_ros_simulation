# sample_docker_carla_project

## Run CARLA Locally

Install Docker Container 
```
docker pull carlasim/carla:latest
```

Create python environment (using conda) 
```
conda env create --name carla python=3.7
conda activate carla
pip install pygame numpy 
```

Copy over python API from container
```
docker cp <container id>:/home/carla/PythonAPI <dest>
```

Launch Carla
```
docker run \
 -p 2000-2002:2000-2002 \
 --cpuset-cpus="0-5" \
 --runtime=nvidia \
 --gpus 'all,"capabilities=graphics,utility,display,video,compute"' \
 -e DISPLAY=$DISPLAY \
 -v /tmp/.X11-unix:/tmp/.X11-unix \
 -it \
 carlasim/carla \
 ./CarlaUE4.sh -opengl $1
```

To interact with Carla using the python API, cd in the local PythonAPI directory copied over. 
Activate the python environment created and you can run scripts from the /examples folder. 

## Run a ROS 2 Humble Container Locally

Pull a container for the appropiate distribution: [Containers](https://hub.docker.com/r/osrf/ros/tags?page=1)
```
docker pull osrf/ros:humble-desktop-full
```

Run container: 
```
docker run -it <image-id>
```

Once inside you can run regular ROS 2 commands:
```
ros2 topic list
ros2 pkg list
```

Run demo nodes:
```
ros2 run demo_nodes_cpp listener & ros2 run demo_nodes_cpp talker
```

Docker-compose demo:
```
docker-compose --file ros2_nodes_demo.yaml up
```

# Memory Usage

From `docker stats`:

```
CONTAINER ID   NAME                                         CPU %     MEM USAGE / LIMIT     MEM %     NET I/O           BLOCK I/O         PIDS
21f61acb0384   ros2_container                               0.05%     40.39MiB / 30.78GiB   0.13%     664kB / 251kB     2.36MB / 852kB    11
fb2b86da4423   foxglove_bridge                              1.34%     59.65MiB / 30.78GiB   0.19%     216MB / 125MB     20.7MB / 700kB    28
24ba8e482ca5   carla_client                                 101.56%   13.07MiB / 30.78GiB   0.04%     4.01MB / 73.3kB   8.76MB / 0B       10
de63bdd20587   carla_ros_bridge                             63.58%    238MiB / 30.78GiB     0.76%     619MB / 217MB     17.2MB / 4.51MB   51
e7f5a5cabd2f   carla_server                                 67.14%    1.98GiB / 30.78GiB    6.43%     1.31MB / 622MB    594MB / 8.19kB    57
728167940c63   sample_docker_carla_project_ros_listener_1   0.11%     21.66MiB / 30.78GiB   0.07%     704kB / 295kB     1.33MB / 4.1kB    12
2befa5505fc0   sample_docker_carla_project_ros_talker_1     0.12%     31.25MiB / 30.78GiB   0.10%     709kB / 320kB     15.2MB / 4.1kB    12
```
