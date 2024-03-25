## Prerequisites
Obviously, you have to have a robot to use Pedro Pathing. Also, Pedro Pathing is only able to work
with omnidirectional drives, like mecanum drive. There is currently no support for swerve drives.
You must also have a localizer of some sort. Pedro Pathing currently has the three-wheel localizer
from Road Runner, but we have plans to create our own localizer soon. Additionally, using
[FTC Dashboard] ()

## Tuning
* To start with, we need the mass of the robot in kg. This is used for the centripetal force
correction, and the mass should be put on line `114` in the `FollowerConstants` class under the
`tuning` package.

* Next, we need to find the preferred mecanum drive vectors. The rollers on mecanum wheels point at a
45 degree angle from the forward direction, but the actual direction the force is output is actually
closer to forward. To find the direction your wheels will go, you will need to run the
`Forward Velocity Tuner` and `Strafe Velocity Tuner` OpModes. These will run your robot at full
power for 40 inches forward and to the right, respectively. The distance can be changed through FTC
Dashboard under the dropdown for each respective class, but higher distances work better. After the
distance has finished running, the end velocity will be output to telemetry. The robot may continue
to drift a little bit after the robot has finished running the distance, so make sure you have
plenty of room. Once you're done, put the velocity for the `Forward Velocity Tuner` on line `25` in
the `FollowerConstants` class, and the velocity for the `Strafe Velocity Tuner` on line `26` in the
`FollowerConstants` class.

* The last set of automatic tuners you'll need to run are the zero power acceleration tuners. These
find the rate at which your robot decelerates when power is cut from the drivetrain. This is used to
get a more accurate estimation of the drive vector. To find this, you will need to run the
`Forward Zero Power Acceleration Tuner` and the `Lateral Zero Power Acceleration Tuner` OpModes.
These will run your robot until it hits a velocity of 10 inches/second forward and to the right,
respectively. The velocity can be changed through FTC Dashboard under the dropdown for each
respective class, but higher velocities work better. After the velocity has been reached, power will
be cut from the drivetrain and the robot's deceleration will be tracked until the robot stops, at
which point it will display the deceleration in telemetry. This robot will need to drift to a stop 
to properly work, and the higher the velocity the greater the drift distance, so make sure you have
enough room. Once you're done, put the zero power acceleration for the
`Forward Zero Power Acceleration Tuner` on line `122` in the `FollowerConstants` class and the zero
power acceleration for the `Lateral Zero Power Acceleration Tuner` on line `127` in the
`FollowerConstants` class.

* After this, we will want to tune the translational PIDs. Go to FTC Dashboard