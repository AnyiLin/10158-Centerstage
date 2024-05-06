package org.firstinspires.ftc.teamcode.pedroPathing.localization.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.PoseUpdater;

/**
 * This is the TurnTuner OpMode.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 5/6/2024
 */
public class TurnTuner extends OpMode {
    private PoseUpdater poseUpdater;

    private Telemetry telemetryA;

    public static double ANGLE = 2 * Math.PI;

    @Override
    public void init() {
        poseUpdater = new PoseUpdater(hardwareMap);
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("Turn your robot " + ANGLE + " radians. Your turn ticks to inches will be shown on the telemetry.");
        telemetryA.update();
    }

    @Override
    public void loop() {
        poseUpdater.update();
        telemetryA.addData("total angle", poseUpdater.getTotalHeading());
        telemetryA.addLine("The multiplier will display what your turn ticks to inches should be to scale your current angle to " + ANGLE + " radians.");
        telemetryA.addData("multiplier", ANGLE / (poseUpdater.getTotalHeading() / poseUpdater.getLocalizer().getTurningMultiplier()));
    }
}
