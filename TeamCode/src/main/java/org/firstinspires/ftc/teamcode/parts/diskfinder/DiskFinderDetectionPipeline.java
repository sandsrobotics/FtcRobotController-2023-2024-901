package org.firstinspires.ftc.teamcode.parts.diskfinder;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.parts.teamprop.TeamPropDetectionPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;

@Config
public class DiskFinderDetectionPipeline extends OpenCvPipeline
{
    /*
     * Some color constants
     */
    static final int ringSat = 50;
    //static final Scalar BLUE = new Scalar(0, 0, 255);
    //static final Scalar YELLOW = new Scalar(0, 255, 255);
    static final Scalar GREEN = new Scalar(60, 255, 255);
    //static final Scalar BLOCK = new Scalar(76,166,40);
    static final Scalar WHITE = new Scalar(255,255,255);
    static public Scalar lower_orange = new Scalar(20,100,30);
    static public Scalar upper_orange = new Scalar(30,255,255);
    /*
     * The core values which define the location and size of the sample regions
     */
    static final double leftTagLeftTopLeft = 50;
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(490,0);
    static final int REGION_WIDTH = 300;
    static final int REGION_HEIGHT = 720;

    /*
     * Points which actually define the sample region rectangles, derived from above values
     *
     * Example of how points A and B work to define a rectangle
     *
     *   ------------------------------------
     *   | (0,0) Point A                    |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                  Point B (70,50) |
     *   ------------------------------------
     *
     */
    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    /*
     * Working variables
     */
    Mat region1;
    Mat inputConv = new Mat();
    Mat extracted = new Mat();
    Mat hsv = new Mat();
    Mat bgr = new Mat();
    int avg1;

    // Volatile since accessed by OpMode thread w/o synchronization
    public volatile Boolean diskIsFound = false;

    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Cb channel to the 'Cb' variable
     */
    void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, inputConv, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(inputConv, extracted, 2);
    }

    void inputToMaskRed(Mat input)
    {
        Scalar lower_red = new Scalar(0,0,200);
        Scalar upper_red= new Scalar(0,0,255);
        Imgproc.cvtColor(input, inputConv, Imgproc.COLOR_BGR2HSV);
        Core.inRange(inputConv, lower_red, upper_red, extracted);
    }

    void inputToMaskOrange(Mat input) {
        //Scalar lower_orange = new Scalar(0, 100, 45);
        //Scalar upper_orange = new Scalar(225, 250, 255);
        Imgproc.cvtColor(input, bgr, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(bgr, hsv, Imgproc.COLOR_BGR2HSV);
        Core.inRange(hsv, lower_orange, upper_orange, extracted );
    }

    void inputToSat(Mat input)
    {
        Imgproc.cvtColor(input, inputConv, Imgproc.COLOR_RGB2HSV);
        Core.extractChannel(inputConv, extracted, 1);
    }

    @Override
    public void init(Mat firstFrame)
    {
        /*
         * We need to call this in order to make sure the 'Cb'
         * object is initialized, so that the submats we make
         * will still be linked to it on subsequent frames. (If
         * the object were to only be initialized in processFrame,
         * then the submats would become delinked because the backing
         * buffer would be re-allocated the first time a real frame
         * was crunched)
         */
        //inputToCb(firstFrame);
        //inputToSat(firstFrame);
        inputToMaskOrange(firstFrame);
        /*
         * Submats are a persistent reference to a region of the parent
         * buffer. Any changes to the child affect the parent, and the
         * reverse also holds true.
         */
        region1 = extracted.submat(new Rect(region1_pointA, region1_pointB));
    }

    @Override
    public Mat processFrame(Mat input)
    {
        //inputToCb(input);
        //inputToSat(input);
        inputToMaskOrange(input);
        /*
         * Compute the average pixel value of each submat region. We're
         * taking the average of a single channel buffer, so the value
         * we need is at index 0. We could have also taken the average
         * pixel value of the 3-channel image, and referenced the value
         * at index 2 here.
         */
        //avg1 = (int) Core.mean(region1).val[0];

        Double hasOrange = Core.sumElems(region1).val[0];

        /*
         * Draw a rectangle showing sample region 1 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        input = extracted;
        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                WHITE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines
        Imgproc.putText(input, // Buffer to draw on
                String.valueOf(hasOrange), // string
                region1_pointA, // position point
                Imgproc.FONT_HERSHEY_SIMPLEX,      // font face
                4,                               // font scale
                WHITE,             // Scalar object for color
                4); // thickness


        if (true) {
            if (hasOrange > 4000000) // Was it from region 1?
            {
                diskIsFound = true; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        5); // Negative thickness means solid fill
            } else {
                diskIsFound = false;
            }
        }
        return input;
    }
    public Boolean getAnalysis()
    {
        return diskIsFound;
    }

    public int getAvg1() {
        return avg1;
    }
    public Boolean getDisk() {
        return diskIsFound;
    }
}

