package org.firstinspires.ftc.teamcode.util.opencv;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TeamPropPipeline extends OpenCvPipeline {

    private Mat output = new Mat(), hsv = new Mat();

    private int WIDTH, HEIGHT, GRAY_ERROR, COLOR;

    private int middleTotal, rightTotal;

    private int rightLeftBound, rightRightBound, rightTopBound, middleLeftBound, middleRightBound, middleTopBound;

    private int minDetected, minMultiple;

    private String navigation = "middle";

    private boolean draw = true;

    public TeamPropPipeline() {
        defaultSetup();
        COLOR = 0;
    }

    public TeamPropPipeline(int color) {
        defaultSetup();
        this.COLOR = color;
    }

    public void defaultSetup() {
        WIDTH = 50 * 2;
        HEIGHT = 60 * 2;
        GRAY_ERROR = 120;
        rightLeftBound = 230;
        rightRightBound = 255;
        rightTopBound = 360;
        middleLeftBound = 200;
        middleRightBound = 285;
        middleTopBound = 85;
        minDetected = 40000;
        minMultiple = 30;
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
        for (int x = 0; x < 640; x++){
            for (int y = 0; y < 480; y++){
                if (y > 480 - 2.2 * (x - 195) && x < 240 && y < 480 - 2.2 * (x - 208)){
                    if (draw){
                        Imgproc.line(input, new Point(x, y), new Point(x + 1, y + 1), new Scalar(255, 255, 255));
                    }
                }
            }
        }
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
            for (int counter2 = middleLeftBound; counter2 < middleRightBound; counter2+=2) {
                if (!(hsv.get(counter, counter2)[1]< GRAY_ERROR)) {
                    middleTotal += (output.get(counter, counter2)[COLOR]);

                    if(draw) Imgproc.line(output, new Point(counter2, counter), new Point(counter2 + 1, counter + 1), new Scalar(255, 255, 255));
                }
            }
        }

        //right column
        rightTotal = 0;
        for (int counter = rightTopBound; counter < rightTopBound + HEIGHT; counter+=3) {
            for (int counter2 = rightLeftBound; counter2 < rightRightBound; counter2+=2) {
                if (!(hsv.get(counter, counter2)[1] < GRAY_ERROR)) {
                    rightTotal += (output.get(counter, counter2)[COLOR]);
                    if (draw)
                        Imgproc.line(output, new Point(counter2, counter), new Point(counter2 + 1, counter + 1), new Scalar(255, 255, 255));

                }

            }
        }


        if (middleTotal > 15000){//rightTotal * minMultiple){
            navigation = "middle";
        } else if (rightTotal > 15000){//middleTotal * minMultiple){
            navigation = "right";
        } else {
            navigation = "left";
        }

        /*
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