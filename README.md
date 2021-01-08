# Visual exploration and path planning via reinforcement learning
Visual exploration and smart data collection via autonomous vehicles is an attractive topic in various disciplines. Disturbances like wind significantly influence both the power consumption of the flying robots and the performance of the camera. This script follows a reinforcement learning approach which combines the effects of the power consumption and the object detection modules to develop a policy for object detection in large areas with limited battery life. The learning model enables dynamic learning of the negative rewards of each action based on the drag forces that is resulted by the motion of the flying robot with respect to the wind field. The algorithm is implemented in a near-real world simulation environment both for the planar motion and flight in different altitudes. The trained agent often performed a trade-off between detecting the objects with high accuracy and increasing the area coverage within its battery life. The developed exploration policy outperformed the complete coverage algorithm by minimizing the traveled path while finding the target objects. The performance of the algorithms under various wind fields was evaluated in planar and 3D motion. During an exploration task with sparsely distributed goals and withing a UAV's battery life, the proposed architecture could detect more than twice the amount of goal objects compared to the coverage path planning algorithm in moderate wind field. In high wind intensities, the energy-aware algorithm could detect 4 times the amount of goal objects when compared to its complete coverage counterpart.


![alt text](https://github.com/AmirNiaraki/Visual-Exploration/blob/master/SimulationRun%20(3).png)


The work is implemented using [Airsim](https://microsoft.github.io/AirSim/) APIs in [Unreal Engine4](https://www.unrealengine.com/en-US/).

Technical detials on network structure and the learning algorithm can be found in our [Arxiv paper](https://arxiv.org/abs/1909.12217).

A demo of the operating network can be found in my [youtube](https://www.youtube.com/watch?v=kea1sEz9NVE&ab_channel=AmirNiaraki). Enjoy!
