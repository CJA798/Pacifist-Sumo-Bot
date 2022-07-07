# Pacifist-Sumo-Bot

This robot was made for a class project called Sumo Bot Battle, in which it got __2nd place__.

The robot is designed to be a pacific robot. It is not designed to attack by itself nor to
intentionally generate any kind of conflict with other robots. This design comes from the
idea that battles can be won without the need of violence. The robot is planned to stay in
position (ideally the center of the ring) and hope for the enemy robot to fail by itself. In
case that doesn’t happen, the robot relies on a tie. Whenever the robot is attacked, at least
one of the bumper sensors will be activated, giving away the location of the enemy robot.
After the location is obtained, the robot will push in that direction until the enemy robot
stops attacking it.

The code and electronics were referenced from [here](https://www.youtube.com/watch?v=k4va1JFtzSg)

The robot has two modes:
* __Sonar ON__: Used for a pre-battle test required on the project specs. The robot activates the sonar, finds a target, and pushes it out of the ring.
* __Bumpers ON__: The robot relies on the bumper sensors __only__. It won't attack nor move unless one or more bumpers are pushed.

_Each mode is activated via a switch that is connected to pin 13 on the Arduino._

![alt text](https://github.com/CJA798/Pacifist-Sumo-Bot/blob/main/SUMOBOT%20V0.0.PNG?raw=true)
