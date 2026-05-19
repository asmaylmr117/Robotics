###### 

### ***Q1***

###### When the robot moves forward, which axis changes (x or y)?

###### Answer: It depends on the robot's heading.

###### 

###### At the very start (no rotation), the robot faces the +X direction, so moving forward changes the X axis. After the first left turn (90°), the robot faces +Y, so forward movement changes the Y axis. In general: whichever axis the robot's nose points along is the one that changes. PyBullet uses a right-handed coordinate system where Z points up.

### **Q2**

###### What happens to the orientation when the robot turns?

###### Answer: The quaternion values change, specifically qz and qw.

###### 

###### When the robot turns left, it rotates around the Z axis (the vertical axis pointing up). The quaternion components qz and qw reflect this rotation. After a 90° left turn: qz increases toward 0.71 and qw decreases toward 0.71. The position (x, y, z) does not change during a pure on-the-spot turn — only the orientation changes.

### **Q3**

###### Did the robot return to the starting position?

###### Answer: Not exactly — there will be a small error offset.

###### 

###### In theory, 4 equal-length sides + 4 × 90° turns = a perfect square, returning to start. In practice:

###### Wheel slip — the wheels slide slightly on the simulated floor.

###### Turn imprecision — we control turns by time, not by angle. A 1.6-second turn ≈ 90° but is never exact.

###### Accumulated error — small errors on each of the 4 sides and 4 turns add up.

###### Simulation step granularity — discrete time steps mean the robot can't make a perfect continuous arc.

###### 

###### The Euclidean distance between start and end will typically be 0.1 – 0.8 metres depending on friction settings.

### **Q4**

###### Can a robot have the same position but different orientation?

###### Answer: Yes, absolutely!

###### 

###### Position and orientation are completely independent. Imagine spinning in place: you stay at the same (x, y, z) point, but your orientation (quaternion) changes continuously. Conversely, a robot can be at the same position but facing a different direction. This is exactly what happens during a turn — the robot rotates without moving, so position stays constant while orientation changes.

###### 

###### Real-world example: A car parked in the same parking spot but facing the opposite direction has the same position, different orientation.

# &#x20;**Bonus – Distance Calculation**

###### Euclidean Distance Formula

###### distance = √( (x₂−x₁)² + (y₂−y₁)² )

###### import math

###### 

###### def euclidean\_distance(pos1, pos2):

###### &#x20;   dx = pos2\[0] - pos1\[0]   # Δx

###### &#x20;   dy = pos2\[1] - pos1\[1]   # Δy

###### &#x20;   return math.sqrt(dx\*\*2 + dy\*\*2)

###### 

###### dist = euclidean\_distance(start\_pos, end\_pos)

###### print(f"Distance from start to end: {dist:.4f} m")

###### This measures the straight-line distance on the ground plane (X-Y) between where the robot started and where it ended up. If the square were perfect, this distance would be 0.

