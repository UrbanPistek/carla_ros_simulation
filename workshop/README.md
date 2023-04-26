# Workshop 

This is a transcript of a workshop to walk you through all aspects of this projects and how to deploy.

This workshops aims cover the following topics: 

1. Docker and Docker Compose
2. Using ROS with Docker
3. Using CARLA with Docker
4. Using Foxglove 
5. Running a Dockerized CARLA + ROS + Robotics Software Simulation Stack

**[Workshop Video](https://youtu.be/a7EO-nI4rdI)**

<br>

## Intro

A copy of the intro slides are availible here: `./ME599_Simulation_Workshop.pdf`

Now, be sure to clone [carla_ros_simulation](https://github.com/UrbanPistek/carla_ros_simulation) 
as it contains everything described in this workshop. 

```
git clone git@github.com:UrbanPistek/carla_ros_simulation.git 
```

**Note:** This workshop is performed on a Linux system (Ubuntu in particular) - it is recommended to use Linux, however, most if not
all componets should be replicable on any OS. 

**Note:** It is assumed you have at minimun Docker and Docker Compose installed, along with access to the Wato Server Cluster. If not 
see the top level `../README.md` for guides on how to install and setup. 
> In general refer to the top level `../README.md` as it contains a comprehensive written guide to everything discussed here.

<br>

## Docker Overview

Run `docker`: 

If you run into a error such as "service not availible": 

[Starting docker](https://docs.docker.com/config/daemon/start/) (if daemon not started): 

```
sudo systemctl start docker
```

Run `docker ps`, you should see: 

```
❯ docker ps
CONTAINER ID   IMAGE     COMMAND   CREATED   STATUS    PORTS     NAMES
```

Run `docker images`.

Get a sample container, `docker pull hello-world`. 

Now run the following: 

```
docker images
docker run hello-world
```

Now with that little demo, lets walk through how to create a simple container.

Navigate to `./app`

You can run and build this container with: 

```
docker build --tag simple_python_app .
docker run simple_python_app
```

In a different terminal, you can see the container running and its stats with: 

```
docker ps
docker stats
```

You can then stop the container with: 

```
docker stop <container_id>
```

<br>

## Docker Compose Overview

Run `docker-compose`. 

Often you will want to run multi-container applications, along with configuring different networking and container settings. 
Docker Compose allows you to do this progammatically, making it easy to track changes and configure specific configurations.

To demonstrate this, we will define a custom docker-compose file - defined as a yaml file. 

Navigate to `./server`

You can build with `docker-compose up`. Press Ctrl+C in the terminal to exit. 

You can build in detached mode with `docker-compose up -d`, then take down using `docker-compose down`.

You use the same docker commands as before to view images and containers: 

```
docker ps
docker stats
docker images
```

<br>

## ROS with Docker

For the rest of the workshop, we will use the scripts and images defined in the rest of the project. 

Navigate to the top level directory of the project: `cd ..`

Now, ROS has docker images of each of its distributions and releases availible that you can pull and use: 

```
cd scripts
docker-compose --file ./ros2_nodes_demo.yaml up
```

We can also go a step further and build our own ROS containers with nodes from a local ROS workspace. 

```
cd ../docker/ros2_container
```

Build the image: 

```
docker build --tag ros2_container .
```

Run bash into container: 

```
docker run -it ros2_container bash 
```

Run example node:

```
cd ros2_ws
ros2 run ros2_utils list_topics_node
```

## CARLA with Docker

There are a couple different ways CARLA can be run with Docker, CARLA provides docker images for release
so its easy to create a `docker-compose` script to launch CARLA. 

```
cd ../../scripts
docker-compose --file ./carla.yaml up
```

## Foxglove Overview

For this demo we will first launch the listener and talker demo nodes: 

```
docker-compose --file ./ros2_nodes_demo.yaml up -d
```

Next, we need to run a Foxglove bridge to stream the information: 

```
docker-compose --file ./foxglove_bridge.yaml up -d
```

Lastly, we want to run foxglove studio to be able to view the information: 

```
docker-compose --file ./foxglove.yaml up -d
```

Now go to `http://localhost:8080` to see all the topics listed. 

<br>

## The Simulation Stack

Everything is defined in a top level `docker-compose.yaml` file at the root of the project directory. 

Navigate to the `README.md` at the root of the project directory and follow the section: `2.0 Deployment`. 


