package org.firstinspires.ftc.teamcode.util;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TeamPropPipeline extends OpenCvPipeline {

    private Mat output, hsv;

    private int width, grayError;

    public TeamPropPipeline() {
        width = 4;
        grayError = 20;
    }

    public TeamPropPipeline(int width, int grayError) {
        this.width = width;
        this.grayError = grayError;
    }

    @Override
    public Mat processFrame(Mat input) {
        output = input.clone();

        hsv = output.clone();
        Imgproc.cvtColor(output, hsv, Imgproc.COLOR_RGB2HSV);

        int leftTotal = 0;

        //left column
        for (int counter = 0; counter < output.size().height; counter++) {
            for (int counter2 = 0; counter2 < width; counter2++) {
                if (!(hsv.get(counter, counter2)[1]<grayError))
                    leftTotal += (output.get(counter, counter2)[0]);
            }
        }
        // writes left column total
        Imgproc.putText(output, leftTotal+"", new Point(0,10), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0,0,255),1);

        int middleTotal = 0;

        //middle column
        for (int counter = 0; counter < output.height(); counter++) {
            for (int counter2 = (int)(output.width()/2.0-width/2.0); counter2 < (int)(output.width()/2.0+width/2.0); counter2++) {
                if (!(hsv.get(counter, counter2)[1]<grayError))
                    middleTotal += (output.get(counter, counter2)[0]);
            }
        }
        // writes middle column total
        Imgproc.putText(output, middleTotal+"", new Point(output.width()/2,10), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0,0,255),1);

        int rightTotal = 0;

        //right column
        for (int counter = 0; counter < output.height(); counter++) {
            for (int counter2 = (int)(output.width()-width); counter2 < output.width(); counter2++) {
                if (!(hsv.get(counter, counter2)[1]<grayError))
                    rightTotal += (output.get(counter, counter2)[0]);
            }
        }

        // writes middle column total
        Imgproc.putText(output, rightTotal+"", new Point(output.width()-70,10), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0,0,255),1);



        // writes which area is the highest
        String navigation = "";
        if (leftTotal == Math.max(leftTotal, Math.max(middleTotal, rightTotal))) navigation = "left";
        if (middleTotal == Math.max(leftTotal, Math.max(middleTotal, rightTotal))) navigation = "middle";
        if (rightTotal == Math.max(leftTotal, Math.max(middleTotal, rightTotal))) navigation = "right";
        Imgproc.putText(output, navigation, new Point(output.width()/2,output.height()-30), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0,0,255),1);

        hsv.release();

        return output;
    }
}
