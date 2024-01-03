package org.firstinspires.ftc.teamcode.parts.teamprop;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TeamPropDetectionPipeline extends OpenCvPipeline
{
    /*
     * An enum to define the team prop position
     */
    public enum TeamPropPosition
    {
        LEFT,
        CENTER,
        RIGHT
    }

    /*
     * Some color constants
     */
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);
    static final Scalar BLOCK = new Scalar(76,166,40);
    static final Scalar WHITE = new Scalar(255,255,255);

    /*
     * The core values which define the location and size of the sample regions
     */
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(0,400);
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(550,350);
    static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(1080,400);
    static final int REGION_WIDTH = 200;
    static final int REGION_HEIGHT = 200;

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
    Point region2_pointA = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x,
            REGION2_TOPLEFT_ANCHOR_POINT.y);
    Point region2_pointB = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region3_pointA = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x,
            REGION3_TOPLEFT_ANCHOR_POINT.y);
    Point region3_pointB = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    /*
     * Working variables
     */
    Mat region1, region2, region3;
    Mat inputConv = new Mat();
    Mat extracted = new Mat();
    int avg1, avg2, avg3;

    // Volatile since accessed by OpMode thread w/o synchronization
    public volatile TeamPropPosition position = TeamPropPosition.LEFT;

    //public TeamPropDetectionPipeline(){}

    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Cb channel to the 'Cb' variable
     */
    void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, inputConv, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(inputConv, extracted, 2);
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
        inputToSat(firstFrame);

        /*
         * Submats are a persistent reference to a region of the parent
         * buffer. Any changes to the child affect the parent, and the
         * reverse also holds true.
         */
        region1 = extracted.submat(new Rect(region1_pointA, region1_pointB));
        region2 = extracted.submat(new Rect(region2_pointA, region2_pointB));
        region3 = extracted.submat(new Rect(region3_pointA, region3_pointB));
    }

    @Override
    public Mat processFrame(Mat input)
    {
        //inputToCb(input);
        inputToSat(input);
        /*
         * Compute the average pixel value of each submat region. We're
         * taking the average of a single channel buffer, so the value
         * we need is at index 0. We could have also taken the average
         * pixel value of the 3-channel image, and referenced the value
         * at index 2 here.
         */
        avg1 = (int) Core.mean(region1).val[0];
        avg2 = (int) Core.mean(region2).val[0];
        avg3 = (int) Core.mean(region3).val[0];
        /*
         * Draw a rectangle showing sample region 1 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                BLOCK, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines
        Imgproc.putText(input, // Buffer to draw on
                String.valueOf(avg1), // string
                region1_pointA, // position point
                Imgproc.FONT_HERSHEY_SIMPLEX,      // font face
                4,                               // font scale
                WHITE,             // Scalar object for color
                4); // tickness

                /*
         * Draw a rectangle showing sample region 2 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                region2_pointA, // First point which defines the rectangle
                region2_pointB, // Second point which defines the rectangle
                BLOCK, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines
        Imgproc.putText(input, // Buffer to draw on
                String.valueOf(avg2), // string
                region2_pointA, // position point
                Imgproc.FONT_HERSHEY_SIMPLEX,      // font face
                4,                               // font scale
                WHITE,             // Scalar object for color
                4); // tickness

        /*
         * Draw a rectangle showing sample region 3 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                region3_pointA, // First point which defines the rectangle
                region3_pointB, // Second point which defines the rectangle
                BLOCK, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines
        Imgproc.putText(input, // Buffer to draw on
                String.valueOf(avg3), // string
                region3_pointA, // position point
                Imgproc.FONT_HERSHEY_SIMPLEX,      // font face
                4,                               // font scale
                WHITE,             // Scalar object for color
                4); // tickness

        /*
         * Find the max of the 3 averages
         */
        int maxOneTwo = Math.max(avg1, avg2);
        int max = Math.max(maxOneTwo, avg3);

        /*
         * Now that we found the max, we actually need to go and
         * figure out which sample region that value was from
         */
        if(max == avg1) // Was it from region 1?
        {
            position = TeamPropPosition.LEFT; // Record our analysis

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
        }
        else if(max == avg2) // Was it from region 2?
        {
            position = TeamPropPosition.CENTER; // Record our analysis

            /*
             * Draw a solid rectangle on top of the chosen region.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    5); // Negative thickness means solid fill
        }
        else if(max == avg3) // Was it from region 3?
        {
            position = TeamPropPosition.RIGHT; // Record our analysis

            /*
             * Draw a solid rectangle on top of the chosen region.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    5); // Negative thickness means solid fill
        }

        /*
         * Render the 'input' buffer to the viewport. But note this is not
         * simply rendering the raw camera feed, because we called functions
         * to add some annotations to this buffer earlier up.
         */
        return input;
    }

    public TeamPropPosition getAnalysis()
    {
        return position;
    }

    public int getAvg1() {
        return avg1;
    }
    public int getAvg2() {
        return avg2;
    }
    public int getAvg3() {
        return avg3;
    }
}
