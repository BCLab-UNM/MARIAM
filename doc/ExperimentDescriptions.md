# Experiments
Document the goals and exact setup of each experiment here. Use as much detail as possible about the weight, dimensions, and distances, etc. of all objects in the scene. 

## Experiment 0.1: Naive Attempt
- Manual operation of the base and arms using a velocity controller

## Experiment 0.2 Naive Attempt
### Monica Setup
```bash
ssh swarmie@monica.local -t /home/swarmie/MARIAM/misc/start_micro_ros_agent.sh
ssh swarmie@monica.local -t /home/swarmie/MARIAM/misc/start_robot.sh
```
### Ross Setup
```bash
ssh swarmie@ross.local -t /home/swarmie/MARIAM/misc/start_micro_ros_agent.sh
ssh swarmie@ross.local -t /home/swarmie/MARIAM/misc/start_robot.sh
```
### Laptop Setup
```bash
ros2 launch mariam_demos laptop_demo1.launch.py 
ros2 run domain_bridge domain_bridge ~/MARIAM/src/mariam_demos/domain_bridge_configs/experiment_1_bridge.yml 
ros2 bag record -a
```
### Physical Setup
- The center of the front tires of both agents are place 1.2 meters apart
- The center of the object is placed at the middle point of the previous distance
- A Styrofoam cube is used as the object
    - Weight: 
    - Length x Width x Height: 8in x 8in x 8in

### Trial Process
Experiment is run via manual control of the base and the arm using MoveIt position controller without path constraints. 
1. Arms are lifted to the height of the object
2. Bases are moved forward until almost touching the object
3. Arms are moved forward until pressure is applied
4. Arms are moved upward until object is help and stable
5. Bases are driven forward about 0.6m and backwards home 0.6m
