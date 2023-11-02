package org.firstinspires.ftc.teamcode.wolfpackPather.localization;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.ArrayList;
import java.util.List;

@Config
public class PoseUpdater extends MecanumDrive {
    private List<Integer> lastEncPositions = new ArrayList<>();
    private List<Integer> lastEncVels = new ArrayList<>();

    private IMU imu;

    public PoseUpdater(HardwareMap hardwareMap) {
        super(0, 0, 0, 0, 0, 0);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        // TODO: if desired, use setLocalizer() to change the localization method
        setLocalizer(new ThreeWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels));
    }

    public void update() {
        updatePoseEstimate();
    }

    @Override
    public List<Double> getWheelVelocities() {
        List returnList = new ArrayList<Double>();
        return returnList;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
    }

    @Override
    public double getRawExternalHeading() {
        return 0.0;//return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return 0.0;//return (double) imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List returnList = new ArrayList<Double>();
        return returnList;
    }
}
