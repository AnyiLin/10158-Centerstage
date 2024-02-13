package org.firstinspires.ftc.teamcode.util.opencv;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TeamPropPipeline extends OpenCvPipeline {

    private Mat output = new Mat(), hsv = new Mat();

    private int HEIGHT, GRAY_ERROR, COLOR;

    private int middleTotal, rightTotal;

    private int rightLeftBound, rightRightBound, rightTopBound, middleLeftBound, middleRightBound, middleTopBound;

    private int minDetected, minMultiple;

    private Scalar WHITE = new Scalar(255, 255, 255);

    private int i, j;

    private String navigation = "middle";

    private boolean draw = true;

    public TeamPropPipeline() {
        defaultSetup();
        COLOR = 2;
    }

    public TeamPropPipeline(int color) {
        defaultSetup();
        this.COLOR = color;
    }

    public void defaultSetup() {
        HEIGHT = 60 * 2;
        GRAY_ERROR = 135;
        rightLeftBound = 255;
        rightRightBound = 290;
        rightTopBound = 360;
        middleLeftBound = 270;
        middleRightBound = 305;
        middleTopBound = 85;
        minDetected = 16000;
        minMultiple = 30;
    }

    // color corresponds with RGB values, with 0 being red, 1 being green, and 2 being blue
    public TeamPropPipeline(int WIDTH, int height, int grayError, int color) {
        this.HEIGHT = height;
        this.GRAY_ERROR = grayError;
        this.COLOR = color;
    }

    @Override
    public Mat processFrame(Mat input) {
        return actualProcessFrame(input);
    }

    public Mat actualProcessFrame(Mat input) {
    if (COLOR == 0) {
        /*
        for (int x = 0; x < 640; x++) {
            for (int y = 0; y < 480; y++) {
                if (y > 480 - 1.9 * (x - 220) && x < 280 && y < 480 - 1.9 * (x - 230)) {
                    Imgproc.line(input, new Point(x, y), new Point(x + 1, y + 1), new Scalar(255, 255, 255));
                }
            }
        }
        for (int x = 0; x < 640; x++) {
            for (int y = 0; y < 480; y++) {
                if (y > 80 && x < 280 && x > 270 && y < 200) {
                    Imgproc.line(input, new Point(x, y), new Point(x + 1, y + 1), new Scalar(255, 255, 255));
                }
            }
        }
         */
    } else if (COLOR == 2) {
        /*
        for (int x = 0; x < 640; x++) {
            for (int y = 0; y < 480; y++) {
                if (y > 480 - 1.9 * (x - 220) && x < 280 && y < 480 - 1.9 * (x - 230)) {
                    Imgproc.line(input, new Point(x, y), new Point(x + 1, y + 1), new Scalar(255, 255, 255));
                }
            }
        }
        for (int x = 0; x < 640; x++) {
            for (int y = 0; y < 480; y++) {
                if (y > 80 && x < 280 && x > 270 && y < 200) {
                    Imgproc.line(input, new Point(x, y), new Point(x + 1, y + 1), new Scalar(255, 255, 255));
                }
            }
        }
         */
    }


        output.release();
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
        for (i = middleTopBound; i < middleTopBound + HEIGHT; i+=3) {
            for (j = middleLeftBound; j < middleRightBound; j+=2) {
                if (!(hsv.get(i, j)[1]< GRAY_ERROR)) {
                    middleTotal += (output.get(i, j)[COLOR]);

                    if(draw) Imgproc.line(output, new Point(j, i), new Point(j + 1, i + 1), WHITE);
                }
            }
        }

        //right column
        rightTotal = 0;
        for (i = rightTopBound; i < rightTopBound + HEIGHT; i+=3) {
            for (j = rightLeftBound; j < rightRightBound; j+=2) {
                if (!(hsv.get(i, j)[1] < GRAY_ERROR)) {
                    rightTotal += (output.get(i, j)[COLOR]);
                    if (draw)
                        Imgproc.line(output, new Point(j, i), new Point(j + 1, i + 1), WHITE);

                }

            }
        }


        if (middleTotal > minDetected){//rightTotal * minMultiple){
            navigation = "middle";
        } else if (rightTotal > minDetected){//middleTotal * minMultiple){
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
        Imgproc.putText(output, navigation+"", new Point(0,80), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, WHITE,1);
        Imgproc.putText(output, middleTotal+"", new Point(0,100), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, WHITE,1);
        Imgproc.putText(output, rightTotal+"", new Point(0,120), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, WHITE,1);
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