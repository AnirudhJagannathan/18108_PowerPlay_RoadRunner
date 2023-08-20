package org.firstinspires.ftc.teamcode.drive.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


/**
 * Created by maryjaneb  on 11/13/2016.
 *
 * nerverest ticks
 * 60 1680
 * 40 1120
 * 20 560
 *
 * monitor: 640 x 480
 *YES
 */
@Autonomous(name= "opencvSkystoneDetector", group="Sky autonomous")
//@Disabled//comment out this line before using
public class opencvSkystoneDetector extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    /*private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;
    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;
    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive
    private static float[] midPos = {4f/8f+offsetX, 2f/8f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f/8f+offsetX, 4f/8f+offsetY};
    private static float[] rightPos = {6f/8f+offsetX, 4f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor
    private final int rows = 640;
    private final int cols = 480;*/



    OpenCvCamera phoneCam;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(this, hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        //P.S. if you're using the latest version of easyopencv, you might need to change the next line to the following:
        //phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);//remove this

        phoneCam.openCameraDevice();//open camera
        //different stages
        StageSwitchingPipeline stageSwitchingPipeline = new StageSwitchingPipeline();
        StageSwitchingPipelineLeftSide stageSwitchingPipelineLeft = new StageSwitchingPipelineLeftSide();
        phoneCam.setPipeline(stageSwitchingPipelineLeft);
        phoneCam.startStreaming(stageSwitchingPipelineLeft.getRows(), stageSwitchingPipelineLeft.getCols(), OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

        waitForStart();
        runtime.reset();

        drive.strafeIMUJunctionLeft(true, 0.2, stageSwitchingPipelineLeft);
    }
}