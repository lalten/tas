#Readme parking-node

The parking procedure is based on the data of the front and back laser. It's divided in three parts:
* passing by the boxes
* backward s-kurve to reach wall
* correction move

##Passing the boxes

For a good fit into the gap between the boxes, the right distance to the wall and a parralel orientation is essential. This is achieved by:
* controller to keep distance to wall
* track boxes passed
* find starting position for backward s-curve

The distance to the wall is based on the front laser. The avarage of a small laser segment is used. There are two distances, one for wall, one for box distance.
The passed boxes are also aquired from the small laser segment, if there is a bis step in distance, it's asumed there is the beginning/end of a box
The final parking position is based on a small laser segment (27,5-29°). This segment looks left behind the car, once it's distracted by the edge of the secound box, the position for the s-curve is reached.

##Backward s-curve

The s-curve is split in two parts. The start of the secound part is defined, by the distance to the wall. Once the distance is unterschritten the servo turn is inversed. The secound end is defined by the distance to the box behind the car.

##correction move.
To put the car in the end position, the car moves forward. To get closer to the wall, the weels are turned left. The final position is reached, when the gap between the car and boxes is equal.

The node has to be closed manually.

Note: With some cars the wii_controll has to be unpaired.
