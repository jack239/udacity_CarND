### Overview
For extend path I used 3 simple steps.
1. Determinate current state.
2. Found main points for trajectory.
3. Create path base current state and points.

### State machine
In this project I used 3 simple state(Actiualy for, but Init state used only one time)
1. Keep lane. In this state there is not car in front of ego car. Ego car moves with max speed.
2. Follow. There is car in front. Ego car moved with speed as this car. Ego car stay on the lane.
3. Change lane. There is car in front. But neighbor lane is free. We tried start moved in this lane.

Default state is keep lane. 
Until another car was detected in small distance. 
Then we start follow this car

In follow state TP check neigbor lanes.
If one of them is free then car start change lane.
Otherwise checked cars in front. 
If no car in front, return to keep lane state else keep follow.

In Change lane state car moved to new lane, until reached it and start keep this new lane. 

### Target points.
In this step I take first 3 points from previous path and used them as start of new.
Also I take one point in middle of target lane in some distance. 
Distance depends of state.

### New path
TP create spline based on 3 points from previous path and target point.
It remains to define the sampling.
For sampling calculation we can use current velocities, current acceleration, target velocities.  
Current velocities, current acceleration are defined by points from previous path.
