# Assignment1 - Research Track I - Serena Paneri 4506977

The assignment requires controlling a holonomic robot in a 2d space with a simple 2d simulator, Stage.
The simulator can be launched by executing the command:

```
rosrun stage_ros stageros $(rospack find assignment1)/world/exercise.world
```

In order to run the content of assignment1 the user have to type two commands,
please do that in two different shells in the following order:

```
rosrun assignment1 target_server.py
```

```
rosrun assignment1 control_robot.py
```

### Expected behaviour:
- 1. The robot asks for a random target, with both coordinates in the interval (-6.0, 6.0)
- 2. The robot reaches the target
- 3. Go to step 1

### Scripts:
The user can visualize two different scripts that are called **target_server.py** and **control_robot.py**.
- The first script, **target_server.py** consists in a server that generates a random position with coordinates (x,y) chosen in a range (min,max) provided by Target.srv.
- The second script, **control_robot.py** consists of controlling the position of the robot in the 2d environment in order to make the robot achiving the random target position.
It uses a communication protocol pubblish/subscribe to share messages and, trough that, the actual position of the robot in the environment is known using the topic odom provided by Odometry.
It has also a client that generates a new target position when the robot reaches the previous one.

### Services:
There is one service called **Target.srv** contained in srv folder. This service requires two float parameters, min and max, which represent the range of coordinates that can be used for the random target position. The response is composed again by two floats which are the x and y coordinates of the random target position that the robot have to reach.

### Nodes:
Assignment1 consists of three nodes.
- **stageros**: this node consists in the 2d space simulator.
- **target_server**: this node communicates with **control_robot** by providing it a new random target position each time that the robot reaches the previous one.
- **control_robot**: this node controls the motion of the robot in the 2d environment provided by **stageros**. It recives the actual position of the robot from the lattest, using the topic **odom**, in order to verify if the robot has actually achieved the random position target. When the distance between the robot and the target position is under a certain pre-determined threshold it modifies the velocity of the robot trough the topic **cmd_vel**.




![rosgraph](https://user-images.githubusercontent.com/93039889/138612308-201e8c50-1acf-488b-978d-d073f74e8c22.png)
