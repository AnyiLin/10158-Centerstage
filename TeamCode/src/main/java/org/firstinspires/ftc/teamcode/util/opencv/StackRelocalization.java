package org.firstinspires.ftc.teamcode.util.opencv;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class StackRelocalization extends OpenCvPipeline {

    private Mat output = new Mat(), hls = new Mat(), mask = new Mat(), dots = new Mat();

    private ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();

    public ArrayList<Rect> boundingBoxes = new ArrayList<Rect>();

    private Rect biggestBox = new Rect();

    private final int CENTER = 73, SEARCH_AREA_X = 160, SEARCH_AREA_UPPER_Y = -5, SEARCH_AREA_LOWER_Y = 80, H_LOWER = 0, L_LOWER = 195, S_LOWER = 0, H_UPPER = 255, L_UPPER = 255, S_UPPER = 255;

    private final double PIXEL_TO_INCH = 1/10;//1/22.5;

    private final Scalar LOWER = new Scalar(H_LOWER,L_LOWER,S_LOWER);
    private final Scalar UPPER = new Scalar(H_UPPER,L_UPPER,S_UPPER);

    private boolean draw = true;

    private double biggestBoxX = CENTER;

    public StackRelocalization() {
    }

    @Override
    public Mat processFrame(Mat input) {
        output.release();
        Imgproc.resize(input, output, input.size());
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
        return getXError()*PIXEL_TO_INCH;
    }
}
