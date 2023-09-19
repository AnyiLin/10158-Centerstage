package org.firstinspires.ftc.teamcode.util;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class NotGoodBackDropPipeline extends OpenCvPipeline {

    @Override
    public Mat processFrame(Mat input) {
        Mat transformedImage = input.clone();

        Point[] bounds = new Point[]{
                new Point(50, 50),
                new Point(440, 0),
                new Point(530, 575),
                new Point(170, 650)
        };

        Mat matrix = Imgproc.getPerspectiveTransform(new MatOfPoint2f(bounds), new MatOfPoint2f(
                new Point(0, 0),
                new Point(640, 0),
                new Point(640, 641),
                new Point(0, 641)
        ));

        Imgproc.warpPerspective(input, transformedImage, matrix, new Size(640, 641));


        List<MatOfPoint> polylines = new ArrayList<>();
        polylines.add(new MatOfPoint(bounds));
        Imgproc.polylines(input, polylines, true, new Scalar(255, 0, 0), 5);

        // Convert the transformed image to HSV color space
        Mat hsvImage = new Mat();
        Imgproc.cvtColor(transformedImage, hsvImage, Imgproc.COLOR_BGR2HSV);

        // Define the lower and upper bounds for the color range
        Scalar lowerBound = new Scalar(0, 0, 170);
        Scalar upperBound = new Scalar(360, 30, 255);

        // Create a binary mask for the specified color range
        Mat range = new Mat();
        Core.inRange(hsvImage, lowerBound, upperBound, range);

        // Find contours in the binary image
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(range, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // Filter contours by area
        List<MatOfPoint> boundedContours = new ArrayList<>();
        double minArea = 200;//450;
        double maxArea = 1000;//850;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > minArea && area < maxArea) {
                boundedContours.add(contour);
            }
        }

        // Draw contours on the transformed image
        Imgproc.drawContours(transformedImage, boundedContours, -1, new Scalar(0, 255, 0), 3);

        Imgproc.resize(transformedImage, transformedImage, new Size (320, 240));

        // releasing all mats
        matrix.release();
        hsvImage.release();
        range.release();
        hierarchy.release();

        return transformedImage;
    }
}
