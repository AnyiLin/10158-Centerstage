package org.firstinspires.ftc.teamcode.wolfpackPather.follower;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.wolfpackPather.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.wolfpackPather.tuning.FollowerConstants;

public class Follower {
    private HardwareMap hardwareMap;
    private DriveVectorScaler driveVectorScaler;

    public Follower(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        driveVectorScaler = new DriveVectorScaler(FollowerConstants.frontLeftVector);
    }

    /**
     * IN A SOMEWHAT ORDERED LIST, HERE ARE THE TODOS
     */

    // TODO: increment along the path by some fixed amount to determine which point is closest to the robot
    // do this by incrementing along the path until the distance from the robot is getting greater
    // also make sure we're not going backwards along the path somehow

    // TODO: make heading pid
    // pretty self explanatory

    // TODO: make path translational correction pid
    // this simply is a pid that corrects towards the closest point on the path from the robot's current position

    // TODO: make centripetal force correction
    // do this by finding the curvature, then using the physics formula to find the force necessary and scaling that to a motor power

    // TODO: measure the deceleration of the drivetrain at 0 power and create the glide-into-position stopping system
    // do this by measuring if the distance from the drivetrain stopping is greater than the path end position
    // then, use a translational pid to move the projected end point onto the actual end point

    // TODO: alternate stopping mechanism?
    // still measure the deceleration, but now measure when the stopping distance is greater
    // than the path end position in terms of arc length
    // then, cut forward drive power, but keep correctional pids
    // alternatively, use a pid to slow down, so when stop distance exceeds arc length remaining,
    // correct backwards, and vice versa
}
