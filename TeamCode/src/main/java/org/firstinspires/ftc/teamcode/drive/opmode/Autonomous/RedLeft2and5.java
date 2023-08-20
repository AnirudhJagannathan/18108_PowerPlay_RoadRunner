package org.firstinspires.ftc.teamcode.drive.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class RedLeft2and5 extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag IDs from the 36h11 family
    int LEFT = 0;
    int MIDDLE = 1;
    int RIGHT = 2;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(this, hardwareMap);

        drive.closeClaw();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        StageSwitchingPipelineLeftSide stageSwitchingPipelineLeftSide = new StageSwitchingPipelineLeftSide();

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        while (opModeInInit()) {
            {
                ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

                if(currentDetections.size() != 0)
                {
                    boolean tagFound = false;

                    for(AprilTagDetection tag : currentDetections)
                    {
                        if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                        {
                            tagOfInterest = tag;
                            tagFound = true;
                            break;
                        }
                    }

                    if (tagFound)
                    {
                        telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                        tagToTelemetry(tagOfInterest);
                    }
                    else
                    {
                        telemetry.addLine("Don't see tag of interest :(");

                        if(tagOfInterest == null)
                        {
                            telemetry.addLine("(The tag has never been seen)");
                        }
                        else
                        {
                            telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                            tagToTelemetry(tagOfInterest);
                        }
                    }

                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }

                }

                telemetry.update();
                sleep(20);
            }
        }

        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest == null) {
            drive.resetSlides();
            drive.resetTurret();

            drive.accelStraightGyroSlidesTurret(12, 0.3,  1000, 0.5, 440, 0.3);
           // drive.accelLeftGyroSlidesTurret(12, 0.3,  1000, 0.5, 440, 0.3, true);
           // drive.accelRightGyroSlidesTurret(12, 0.3, 1000, 0.5, 440, 0.3, true);
        }

        else if (tagOfInterest != null) {
            drive.resetTurret();
            drive.resetSlides();

            drive.closeClaw();
            camera.setPipeline(stageSwitchingPipelineLeftSide);

            drive.accelStraightGyroSlidesTurret(39, 0.45, 1500, 0.8, 380,0.3);
            drive.accelLeftGyroSlidesTurret(2.5, 0.2,1500, 0, 380, 0.3);
            drive.moveSlidesToHeightABS(1300,0.8);
            drive.openClaw(); //Drop first cone in the junction
            sleep(500);
            drive.moveSlidesToHeightABS(1700,0.8);
            drive.accelRightGyroSlidesTurret(3, 0.2,1700, 0, 380, 0);

            //sleep(2000);
            drive.accelStraightGyroSlidesTurret(15, 0.5, 1700, 0, 380,0);
            sleep(300);
            drive.accelStraightGyroSlidesTurret(-4, 0.4, 1700, 0, 380,0);
            sleep(300);
            drive.moveSlidesToHeightABS(780,0.8);
            drive.accelLeftGyroSlidesTurret(20, 0.8, 750, 0, 380, 0);
            sleep(200);
            //drive.coneLineDetect(true, true);
            drive.accelLeftGyroSlidesTurret(6.5, 0.3, 750, 0, 380, 0);
            sleep(200);
            drive.closeClaw();
            sleep(200);
            drive.moveSlidesToHeightABS(1500, 0.8);
            sleep(200);
            drive.accelRightGyroSlidesTurret(6, 0.8, 1500, 0.2, 380, 0);
            sleep(2000);
            drive.accelRightGyroSlidesTurret(12, 0.4, 600, 0, 880,0.3);
            //drive.strafeIMUJunctionLeft(false, 0.2, stageSwitchingPipelineLeftSide);


            /* sleep(2000);
            drive.accelStraightGyroSlidesTurret(-3.5, 0.2, 2500, 0.9, 880, 0);
            sleep(200);
            drive.moveSlidesToHeightABS(2450, 0.2);
            drive.openClaw();
            sleep(200);
            //Drop the second cone on the medium junction

             */

            /*
            drive.accelStraightGyroSlidesTurret(2, 0.2, 2450, 0, 440, 0.4);
            drive.accelLeftGyroSlidesTurret(40, 0.75, 500, 0.7, 440, 0.4, true);
            drive.accelStraightGyroSlidesTurret(0, 0, 470, 0, 440, 0.4);
            drive.coneLineDetect(true, true);
            drive.accelLeftGyroSlidesTurret(1, 0.3, 470, 0, 440, 0, false);
            drive.closeClaw();
            sleep(200);
            //Pick up the third cone
             */

            drive.moveSlidesToHeightABS(1100, 0.7);
            drive.turnTankGyro(-3, 0.3);
            drive.accelRightGyroSlidesTurret(40, 0.8, 1100, 0.2, 880, 0.4);
            drive.strafeIMUJunctionLeft(false, 0.2, stageSwitchingPipelineLeftSide);
            drive.accelStraightGyroSlidesTurret(-4, 0.2, 2500, 0.9, 880, 0);
            sleep(200);
            drive.moveSlidesToHeightABS(2450, 0.4);
            drive.openClaw();
            //Drop the third cone

            drive.accelStraightGyroSlidesTurret(1.5, 0.3, 2400, 0, 410, 0.3);
            drive.closeClaw();


            //Move to the medium junction from the start and drop the cone

            if(tagOfInterest.id == LEFT)
                drive.accelLeftGyroSlidesTurret(39, 0.5, 200, 0.6, 0, 0.6);
                //Move to square one

            else if(tagOfInterest.id == MIDDLE)
                drive.accelLeftGyroSlidesTurret(14, 0.5, 200, 0.6, 0, 0.6);
                //Move to square two

            else if(tagOfInterest.id == RIGHT) {
                drive.accelRightGyroSlidesTurret(12, 0.8, 200, 0.4, 0, 0.7);
            }
            //Move to square three
        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
