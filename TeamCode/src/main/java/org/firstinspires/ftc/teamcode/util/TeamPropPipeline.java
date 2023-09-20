package org.firstinspires.ftc.teamcode.util;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TeamPropPipeline extends OpenCvPipeline {

    private Mat output, hsv;

    private final int WIDTH, HEIGHT, GRAY_ERROR, COLOR;

    private int leftTotal, middleTotal, rightTotal;

    private String navigation;

    public TeamPropPipeline() {
        WIDTH = 4;
        HEIGHT = 80;
        GRAY_ERROR = 20;
        COLOR = 0;
    }

    public TeamPropPipeline(int color) {
        WIDTH = 4;
        HEIGHT = 80;
        GRAY_ERROR = 20;
        this.COLOR = color;
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
        output = input.clone();

        hsv = output.clone();
        Imgproc.cvtColor(output, hsv, Imgproc.COLOR_RGB2HSV);

        //left column
        leftTotal = 0;
        for (int counter = output.height()- HEIGHT; counter < output.height(); counter++) {
            for (int counter2 = 0; counter2 < WIDTH; counter2++) {
                if (!(hsv.get(counter, counter2)[1]< GRAY_ERROR))
                    leftTotal += (output.get(counter, counter2)[COLOR]);
            }
        }

        //middle column
        middleTotal = 0;
        for (int counter = output.height()- HEIGHT; counter < output.height(); counter++) {
            for (int counter2 = (int)(output.width()/2.0- WIDTH /2.0); counter2 < (int)(output.width()/2.0+ WIDTH /2.0); counter2++) {
                if (!(hsv.get(counter, counter2)[1]< GRAY_ERROR))
                    middleTotal += (output.get(counter, counter2)[COLOR]);
            }
        }

        //right column
        rightTotal = 0;
        for (int counter = output.height()- HEIGHT; counter < output.height(); counter++) {
            for (int counter2 = (int)(output.width()- WIDTH); counter2 < output.width(); counter2++) {
                if (!(hsv.get(counter, counter2)[1]< GRAY_ERROR))
                    rightTotal += (output.get(counter, counter2)[COLOR]);
            }
        }

        if (leftTotal == Math.max(leftTotal, Math.max(middleTotal, rightTotal))) navigation = "left";
        if (middleTotal == Math.max(leftTotal, Math.max(middleTotal, rightTotal))) navigation = "middle";
        if (rightTotal == Math.max(leftTotal, Math.max(middleTotal, rightTotal))) navigation = "right";

        writeOnScreen();

        hsv.release();

        return output;
    }

    public String getNavigation() {
        return navigation;
    }

    public void writeOnScreen() {
        // writes left column total
        Imgproc.putText(output, leftTotal+"", new Point(0,10), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0,0,255),1);
        // writes middle column total
        Imgproc.putText(output, middleTotal+"", new Point(output.width()/2,10), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0,0,255),1);
        // writes right column total
        Imgproc.putText(output, rightTotal+"", new Point(output.width()-70,10), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0,0,255),1);
        // writes which area is the highest
        Imgproc.putText(output, navigation, new Point(output.width()/2,output.height()-30), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0,0,255),1);
    }
}
