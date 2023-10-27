package org.firstinspires.ftc.teamcode.util;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.util.opencv.TeamPropPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class VisionPortalTeamPropPipeline implements VisionProcessor {

    private TeamPropPipeline teamPropPipeline;

    public VisionPortalTeamPropPipeline() {
        teamPropPipeline = new TeamPropPipeline();
    }

    public VisionPortalTeamPropPipeline(int color) {
        teamPropPipeline = new TeamPropPipeline(color);
    }

    public VisionPortalTeamPropPipeline(int WIDTH, int height, int grayError, int color) {
        teamPropPipeline = new TeamPropPipeline(WIDTH, height, grayError, color);
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        return teamPropPipeline.actualProcessFrame(frame);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public String getNavigation() {
        return teamPropPipeline.getNavigation();
    }
}
