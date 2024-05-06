package org.firstinspires.ftc.teamcode.pedroPathing.localization.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.PoseUpdater;

/**
 * This is the ForwardTuner OpMode.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 5/6/2024
 */
public class FowardTuner extends OpMode {
    private PoseUpdater poseUpdater;

    private Telemetry telemetryA;

    public static double DISTANCE = 30;

    @Override
    public void init() {
        poseUpdater = new PoseUpdater(hardwareMap);
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("Pull your robot forward " + DISTANCE + " inches. Your forward ticks to inches will be shown on the telemetry.");
        telemetryA.update();
    }

    @Override
    public void loop() {
        poseUpdater.update();
        telemetryA.addData("distance moved", poseUpdater.getPose().getX());
        telemetryA.addLine("The multiplier will display what your forward ticks to inches should be to scale your current distance to " + DISTANCE + " inches.");
        telemetryA.addData("multiplier", DISTANCE / (poseUpdater.getPose().getX() / poseUpdater.getLocalizer().getForwardMultiplier()));
    }
}
