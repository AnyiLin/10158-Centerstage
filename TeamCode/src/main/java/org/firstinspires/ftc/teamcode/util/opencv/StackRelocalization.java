package org.firstinspires.ftc.teamcode.util.opencv;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;

public class StackRelocalization extends OpenCvPipeline {

    public Mat output = new Mat(), hls = new Mat(), mask = new Mat(), dots = new Mat(), undistorted = new Mat(), cameraMatrix, optimalCameraMatrix;

    private MatOfDouble distCoeffs = new MatOfDouble(new double[]{-0.0449369, 1.17277, 0, 0, -3.63244, 0, 0, 0});

    // Lens intrinsics
    // UNITS ARE PIXELS
    private double fx = 822.317;
    private double fy = 822.317;
    private double cx = 319.495;
    private double cy = 242.502;

    private ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();

    public ArrayList<Rect> boundingBoxes = new ArrayList<Rect>();

    private Rect biggestBox = new Rect();

    public int CENTER = 220, SEARCH_AREA_X = 440, SEARCH_AREA_UPPER_Y = -80, SEARCH_AREA_LOWER_Y = 80*2, H_LOWER = 0, L_LOWER = 180, S_LOWER = 0, H_UPPER = 255, L_UPPER = 255, S_UPPER = 255;

    private final double PIXEL_OBJECT_WIDTH_INCHES = 3.34646;

    private double pixelToInch = (double)1/20;//1/22.5;

    private final Scalar LOWER = new Scalar(H_LOWER,L_LOWER,S_LOWER);
    private final Scalar UPPER = new Scalar(H_UPPER,L_UPPER,S_UPPER);

    private boolean draw = true;

    private double biggestBoxX = CENTER;

    public StackRelocalization() {
        constructCameraMatrices();
    }

    @Override
    public Mat processFrame(Mat input) {
        return actualProcessFrame(input);
    }

    public Mat actualProcessFrame(Mat input) {
        output.release();
        Imgproc.resize(input, output, input.size());

        undistortImage();

        Imgproc.rectangle(output, new Point(0,0),new Point(CENTER-SEARCH_AREA_X/2, output.height()), new Scalar(0,0,0),-1);
        Imgproc.rectangle(output, new Point(CENTER+SEARCH_AREA_X/2,0),new Point(output.width(), output.height()), new Scalar(0,0,0),-1);
        Imgproc.rectangle(output, new Point(CENTER-SEARCH_AREA_X/2,0),new Point(CENTER+SEARCH_AREA_X/2, output.height()/2-SEARCH_AREA_UPPER_Y), new Scalar(0,0,0),-1);
        Imgproc.rectangle(output, new Point(CENTER-SEARCH_AREA_X/2,output.height()/2+SEARCH_AREA_LOWER_Y),new Point(CENTER+SEARCH_AREA_X/2, output.height()), new Scalar(0,0,0),-1);

        Imgproc.cvtColor(output, hls, Imgproc.COLOR_RGB2HLS);

        Core.inRange(hls, LOWER, UPPER, mask);

        Imgproc.findContours(mask, contours, dots, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours) {
            boundingBoxes.add(Imgproc.boundingRect(contour));
        }

        findBiggestBox();

        if (draw) writeOnScreen();

        getBiggestBoundingBoxX();

        hls.release();
        dots.release();
        contours.clear();
        boundingBoxes.clear();
        mask.release();
        undistorted.release();

        return output;
    }

    public void writeOnScreen() {
        Imgproc.drawContours(output, contours, -1, new Scalar(255,0,0));
        for (Rect box : boundingBoxes) {
            Imgproc.rectangle(output, box, new Scalar(0, 255, 0));
        }
        if (!boundingBoxes.isEmpty()) Imgproc.rectangle(output, biggestBox, new Scalar(255, 255, 0));
        Imgproc.line(output, new Point(CENTER, 0), new Point(CENTER, output.height()), new Scalar(0,0,0));
        Imgproc.putText(output, "error: "+getXError(), new Point(0,120), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(255,255,255),1);
        if (!boundingBoxes.isEmpty()) Imgproc.putText(output, "box width: "+biggestBox.width, new Point(0,180), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(255,255,255),1);
    }

    public void getBiggestBoundingBoxX() {
        if (!boundingBoxes.isEmpty()) biggestBoxX = biggestBox.x+biggestBox.width/2;
    }

    public void findBiggestBox() {
        if (!boundingBoxes.isEmpty()) {
            biggestBox = boundingBoxes.get(0);
            for (Rect box : boundingBoxes) {
                if (box.area() > biggestBox.area()) {
                    biggestBox = box;
                }
            }
        }
    }

    public double getXError() {
        return biggestBoxX - CENTER;
    }

    public double getXErrorInches() {
        if (!biggestBox.empty()) pixelToInch = 1.0/(biggestBox.width/PIXEL_OBJECT_WIDTH_INCHES);
        return getXError()* pixelToInch;
    }

    public void undistortImage() {
        Calib3d.undistort(output, undistorted, cameraMatrix, distCoeffs, optimalCameraMatrix);
        Imgproc.resize(undistorted, output, undistorted.size());
    }

    public String getCenterHue(){
        return Arrays.toString(hls.get(hls.height()/2,CENTER));

    }

    public void constructCameraMatrices() {
        //     Construct the camera matrix.
        //
        //      --         --
        //     | fx   0   cx |
        //     | 0    fy  cy |
        //     | 0    0   1  |
        //      --         --
        //

        cameraMatrix = new Mat(3,3, CvType.CV_32FC1);

        cameraMatrix.put(0,0, fx);
        cameraMatrix.put(0,1,0);
        cameraMatrix.put(0,2, cx);

        cameraMatrix.put(1,0,0);
        cameraMatrix.put(1,1,fy);
        cameraMatrix.put(1,2,cy);

        cameraMatrix.put(2, 0, 0);
        cameraMatrix.put(2,1,0);
        cameraMatrix.put(2,2,1);
        optimalCameraMatrix = Calib3d.getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, new Size(320, 240), 1);
    }

}
