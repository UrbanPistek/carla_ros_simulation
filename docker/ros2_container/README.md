# info

Defines a example of a ROS container which copies over a ROS workspace and builds and runs the nodes in the container. 

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

