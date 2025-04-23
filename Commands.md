# Commands

The commands currently available, reported as you have to write them in the *command publisher* node, are:

1. *start*: the **first** command to be executed, starts the stream of healthy messages necessary to control the drone;
2. *offboard*: the **second** command to be executed but not mandatory if you can switch flight mode in other way, it has to be sent at least 1 second later than the *start* command, this command sets the firmware into offboard mode, is the system is not in this mode, it won't execute any external command;
3. *arm*: the **third** command, arms the vehicle;
4. *go*: set a new destination for the drone, the coordinates passed have to be in an ENU frame;
5. *w* or *wait*: stops the drone at its current position;
6. *yaw*: changes drone yaw angle, the angle passed have to be in degrees and in an ENU frame;
7. *mission*: starts a small mission, a sequence of points to reach with a specific orientation;
8. *land*: executes a landing maneuver;
9. *k* or *kill*: forces the disarm of the drone;
10. *disarm*: disarms the drone, this command is executed only if the firmware detects to be on the ground;
11. *stop*: lands the drone and stops the stream of healthy messages.

## ***START, OFFBOARD*** AND ***ARM*** must be the first three commands 
