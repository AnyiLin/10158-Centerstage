package org.firstinspires.ftc.teamcode.util.opencv;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class ColorDetection extends OpenCvPipeline {

    private Mat output, hsv;

    private ArrayList<Double> averageR, averageG, averageB, averageH, averageS, averageV;

    private int averageAmount = 10;

    public ColorDetection() {
        averageR = new ArrayList<Double>();
        averageG = new ArrayList<Double>();
        averageB = new ArrayList<Double>();
        averageH = new ArrayList<Double>();
        averageS = new ArrayList<Double>();
        averageV = new ArrayList<Double>();
    }

    @Override
    public Mat processFrame(Mat input) {

        //input.convertTo(input, -1, 2.0, -30.0); // Increase contrast and adjust brightness

        output = input.clone();

        hsv = output.clone();
        Imgproc.cvtColor(output, hsv, Imgproc.COLOR_RGB2HSV);


        updateAverage();

        writeOnScreen();

        hsv.release();

        return output;
    }
    public void writeOnScreen() {
        //draws middle cursor
        Imgproc.line(output,new Point(output.width()/2-5,output.height()/2), new Point(output.width()/2+5, output.height()/2), new Scalar(0,0,0));
        //draws middle cursor
        Imgproc.line(output,new Point(output.width()/2,output.height()/2-5), new Point(output.width()/2, output.height()/2+5), new Scalar(0,0,0));
        // writes writes middle pixel color
        Imgproc.putText(output, "hsv: "+getAverage(averageH)+", "+getAverage(averageS)+", "+getAverage(averageV), new Point(0,output.height()-30), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0,0,0),1);
        Imgproc.putText(output, "rgb: "+getAverage(averageR)+", "+getAverage(averageG)+", "+getAverage(averageB), new Point(0,output.height()-15), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0,0,0),1);
        Imgproc.putText(output, "brightness: "+(getAverage(averageR)+getAverage(averageG)+getAverage(averageB))/3, new Point(0,output.height()-45), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0,0,0),1);

    }

    public double getAverage(ArrayList<Double> get) {
        double avg = 0;
        for (double num : get) {
            avg += num;
        }
        return avg/get.size();
    }

    public void updateAverage() {
        averageR.add(output.get(output.height()/2, output.width()/2)[0]);
        if (averageR.size() > averageAmount) {
            averageR.remove(0);
        }
        averageG.add(output.get(output.height()/2, output.width()/2)[1]);
        if (averageG.size() > averageAmount) {
            averageG.remove(0);
        }
        averageB.add(output.get(output.height()/2, output.width()/2)[2]);
        if (averageB.size() > averageAmount) {
            averageB.remove(0);
        }
        averageH.add(hsv.get(hsv.height()/2, hsv.width()/2)[0]);
        if (averageH.size() > averageAmount) {
            averageH.remove(0);
        }
        averageS.add(hsv.get(hsv.height()/2, hsv.width()/2)[1]);
        if (averageS.size() > averageAmount) {
            averageS.remove(0);
        }
        averageV.add(hsv.get(hsv.height()/2, hsv.width()/2)[2]);
        if (averageV.size() > averageAmount) {
            averageV.remove(0);
        }
    }
}
