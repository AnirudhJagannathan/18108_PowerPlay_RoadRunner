package org.firstinspires.ftc.teamcode.drive.opmode.Autonomous;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class StageSwitchingPipeline extends OpenCvPipeline
{
    private int valMid;
    private int valLeft;
    private int valRight;

    float rectHeight = 6f/8f;
    float rectWidth = 2f/8f;

    private static float offsetX = 6f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {0f/8f+offsetX, 0f/8f+offsetY};//0 = col, 1 = row
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private int rows;
    private int cols;

    public StageSwitchingPipeline() {
        valMid = -1;
        valLeft = -1;
        valRight = -1;
        cols = 640;
        rows = 480;
    }

    OpenCvCamera phoneCam;
    Mat yCbCrChan2Mat = new Mat();
    Mat thresholdMat = new Mat();
    Mat all = new Mat();
    List<MatOfPoint> contoursList = new ArrayList<>();

    enum Stage
    {//color difference. greyscale
        detection,//includes outlines
        THRESHOLD,//b&w
        RAW_IMAGE,//displays raw view
    }

    private Stage stageToRenderToViewport = Stage.detection;
    private Stage[] stages = Stage.values();

    public int getValMid() {
        return valMid;
    }

    public int getValLeft() {
        return valLeft;
    }

    public int getValRight(){
        return valRight;
    }

    public int getRows(){
        return cols;
    }

    public int getCols(){
        return rows;
    }

    public void setValMid(int n) {
        valMid = n;
    }

    @Override
    public void onViewportTapped()
    {
        /*
         * Note that this method is invoked from the UI thread
         * so whatever we do here, we must do quickly.
         */

        int currentStageNum = stageToRenderToViewport.ordinal();

        int nextStageNum = currentStageNum + 1;

        if(nextStageNum >= stages.length)
        {
            nextStageNum = 0;
        }

        stageToRenderToViewport = stages[nextStageNum];
    }

    @Override
    public Mat processFrame(Mat input)
    {
        contoursList.clear();
        /*
         * This pipeline finds the contours of yellow blobs such as the Gold Mineral
         * from the Rover Ruckus game.
         */

        //color diff cb.
        //lower cb = more blue = skystone = white
        //higher cb = less blue = yellow stone = grey
        Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
        Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

        //b&w
        Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

        //outline/contour
        Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        yCbCrChan2Mat.copyTo(all);//copies mat object
        //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


        //get values from frame
        double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
        valMid = (int)pixMid[0];


        //create three points
        Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));

        //draw circles on those points
        Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle

        //draw 3 rectangles
        Imgproc.rectangle(//3-5
                all,
                new Point(
                        input.cols()*(midPos[0]-rectWidth/2),
                        input.rows()*(midPos[1]-rectHeight/2)),
                new Point(
                        input.cols()*(midPos[0]+rectWidth/2),
                        input.rows()*(midPos[1]+rectHeight/2)),
                new Scalar(0, 255, 0), 3);

        switch (stageToRenderToViewport)
        {
            case THRESHOLD:
            {
                return thresholdMat;
            }

            case detection:
            {
                return all;
            }

            default:
            {
                return input;
            }
        }
    }

}