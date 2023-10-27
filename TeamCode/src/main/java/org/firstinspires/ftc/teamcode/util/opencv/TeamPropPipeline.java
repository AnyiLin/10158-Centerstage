package org.firstinspires.ftc.teamcode.util.opencv;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TeamPropPipeline extends OpenCvPipeline {

    private Mat output = new Mat(), hsv = new Mat();

    private final int WIDTH, HEIGHT, GRAY_ERROR, COLOR;

    private int middleTotal, rightTotal;

    private int rightLeftBound, rightTopBound, middleLeftBound, middleTopBound;

    private int minDetected;

    private String navigation = "middle";

    private boolean draw = true;

    public TeamPropPipeline() {
        WIDTH = 50 * 2;
        HEIGHT = 60 * 2;
        GRAY_ERROR = 120;
        rightLeftBound = 200 * 2;
        rightTopBound = 110 * 2;
        middleLeftBound = 50 * 2;
        middleTopBound = 110 * 2;
        minDetected = 15000 * 2;

        COLOR = 0;
    }

    public int temp;

    public TeamPropPipeline(int color) {
        WIDTH = 50 * 2;
        HEIGHT = 60 * 2;
        GRAY_ERROR = 120;
        this.COLOR = color;
        rightLeftBound = 200 * 2;
        rightTopBound = 100 * 2;
        middleLeftBound = 50 * 2;
        middleTopBound = 110 * 2;
        minDetected = 10000 * 2;

    }

    // color corresponds with RGB values, with 0 being red, 1 being green, and 2 being blue
    public TeamPropPipeline(int WIDTH, int height, int grayError, int color) {
        this.WIDTH = WIDTH;
        this.HEIGHT = height;
        this.GRAY_ERROR = grayError;
        this.COLOR = color;
    }

    @Override
    public Mat processFrame(Mat input) {
        return actualProcessFrame(input);
    }

    public Mat actualProcessFrame(Mat input) {
        output = input.clone();

        Imgproc.cvtColor(output, hsv, Imgproc.COLOR_RGB2HSV);

        //middle column
        /*
        middleTotal = 0;
        for (int counter = middleTopBound; counter < middleTopBound + HEIGHT; counter+=3) {
            for (int counter2 = middleLeftBound; counter2 < middleLeftBound + WIDTH; counter2+=2) {
                if (!(hsv.get(counter, counter2)[1]< GRAY_ERROR)) {
                    middleTotal += (output.get(counter, counter2)[COLOR]);

                    if(draw) Imgproc.line(output, new Point(counter2, counter), new Point(counter2 + 1, counter + 1), new Scalar(255, 255, 255));
                }
            }
        }*/

        //middle column
        middleTotal = 0;
        for (int counter = middleTopBound; counter < middleTopBound + HEIGHT; counter+=3) {
            for (int counter2 = middleLeftBound; counter2 < middleLeftBound + WIDTH; counter2+=2) {
                if (!(hsv.get(counter, counter2)[1]< GRAY_ERROR)) {
                    middleTotal += (output.get(counter, counter2)[COLOR]);

                    if(draw) Imgproc.line(output, new Point(counter2, counter), new Point(counter2 + 1, counter + 1), new Scalar(255, 255, 255));
                }
            }
        }

        //right column
        rightTotal = 0;
        for (int counter = rightTopBound; counter < rightTopBound + HEIGHT; counter+=3) {
            for (int counter2 = rightLeftBound; counter2 < rightLeftBound + WIDTH; counter2+=2) {
                if (!(hsv.get(counter, counter2)[1] < GRAY_ERROR)) {
                    rightTotal += (output.get(counter, counter2)[COLOR]);
                    if (draw)
                        Imgproc.line(output, new Point(counter2, counter), new Point(counter2 + 1, counter + 1), new Scalar(255, 255, 255));

                }

            }
        }
        if (middleTotal > minDetected || rightTotal > minDetected){
            if (rightTotal == Math.max(rightTotal, middleTotal)) navigation = "right";
            if (middleTotal == Math.max(rightTotal, middleTotal)) navigation = "middle";
        } else {
            navigation = "left";
        }/*
        if (leftTotal == Math.max(leftTotal, Math.max(middleTotal, rightTotal))) navigation = "left";
        if (middleTotal == Math.max(leftTotal, Math.max(middleTotal, rightTotal))) navigation = "middle";
        if (rightTotal == Math.max(leftTotal, Math.max(middleTotal, rightTotal))) navigation = "right";*/

        if (draw) writeOnScreen();

        hsv.release();

        return output;
    }

    public String getNavigation() {
        return navigation;
    }

    public void writeOnScreen() {
        Imgproc.putText(output, navigation+"", new Point(0,80), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(255,255,255),1);
        Imgproc.putText(output, middleTotal+"", new Point(0,100), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(255,255,255),1);
        Imgproc.putText(output, rightTotal+"", new Point(0,120), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(255,255,255),1);
        /*
        // writes left column total
        Imgproc.putText(output, leftTotal+"", new Point(0,10), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(255,255,255),1);
        // writes middle column total
        Imgproc.putText(output, middleTotal+"", new Point(output.width()/2,10), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(255,255,255),1);
        // writes right column total
        Imgproc.putText(output, rightTotal+"", new Point(output.width()-70,10), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(255,255,255),1);
        // writes which area is the highest
        Imgproc.putText(output, navigation, new Point(output.width()/2,output.height()-30), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0,0,0),1);*/
    }
}
