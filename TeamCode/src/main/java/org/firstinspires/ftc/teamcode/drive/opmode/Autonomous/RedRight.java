package org.firstinspires.ftc.teamcode.drive.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.JUNCTION_LOW;

import org.firstinspires.ftc.teamcode.drive.opmode.Autonomous.StageSwitchingPipeline;
import org.firstinspires.ftc.teamcode.drive.opmode.Autonomous.StageSwitchingPipelineLeftSide;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class RedRight extends LinearOpMode {
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
        StageSwitchingPipeline stageSwitchingPipeline = new StageSwitchingPipeline();
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

            // drive.driveStraightGyroSlidesTurret(2, 0.3, 1000, 0.5, 0, 0);
        }

        else if (tagOfInterest != null) {
            drive.resetTurret();
            drive.resetSlides();
            drive.resetAngle();

            drive.closeClaw();
            camera.setPipeline(stageSwitchingPipeline);

            drive.accelStraightGyroSlidesTurret(36.5, 0.6, 900, 0.8, 440,0.3);
            sleep(100);
            drive.moveIMUJunction(0.15, stageSwitchingPipeline);
            drive.accelLeftGyroSlidesTurret(2, 0.3, 2550, 0.7, 440, 0);
            sleep(100);
            drive.moveSlidesToHeightABS(2300, 0.3);
            drive.openClaw();
            //Drop cone on the first medium junction

            drive.accelStraightGyroSlidesTurret(14.5, 0.65, 2000, 0.4, 0,0.3);
            sleep(150);
            drive.driveStraightGyroSlidesTurret(-4.7, 0.55, 2000, 0, 0, 0.3);
            drive.accelRightGyroSlidesTurret(30.25, 0.6, 650, 0.5, -420, 0.3);
            drive.closeClaw();
            sleep(200);
            drive.moveSlidesToHeightABS(1100, 0.6);
            drive.accelLeftGyroSlidesTurret(10.5, 0.6, 1700, 0.5, -880, 0.4);
            drive.accelStraightGyroSlidesTurret(-4, 0.45, 1600, 0.4, -880, 0.0);
            drive.moveSlidesToHeightABS(1200, 0.4);
            drive.openClaw();
            sleep(150);

            drive.driveStraightGyroSlidesTurret(2.5, 0.3, 550, 0.4, -420, 0.3);
            drive.accelRightGyroSlidesTurret(11, 0.6, 550, 0, -420, 0.4);
            sleep(100);
            drive.closeClaw();
            sleep(200);
            drive.moveSlidesToHeightABS(1250, 0.65);
            drive.accelLeftGyroSlidesTurret(39.5, 0.8, 1050, 0.2, -880, 0.3);

            camera.setPipeline(stageSwitchingPipelineLeftSide);
            drive.strafeIMUJunctionLeft(true, 0.2, stageSwitchingPipelineLeftSide);
            camera.setPipeline(stageSwitchingPipeline);
            drive.accelStraightGyroSlidesTurret(-3, 0.2, 2550, 0.9, -880, 0);
            drive.moveSlidesToHeightABS(2300, 0.3);
            drive.openClaw();

            drive.driveStraightGyroSlidesTurret(2, 0.3, 450, 0.4, -420, 0.3);
            drive.turnTankGyro(-2.5, 0.3);
            drive.accelRightGyroSlidesTurret(40.5, 0.85, 450, 0, -420, 0.4);
            sleep(100);
            drive.closeClaw();
            sleep(200);
            drive.moveSlidesToHeightABS(1200, 0.5);
            drive.accelLeftGyroSlidesTurret(36.5, 0.8, 2000, 0.4, 0, 0.3);

            drive.strafeIMUJunction(true, 0.2, stageSwitchingPipeline);
            drive.accelStraightGyroSlidesTurret(2, 0.2, 3550, 0.9, 0, 0);
            drive.moveSlidesToHeightABS(3450, 0.3);
            drive.openClaw();
            drive.accelStraightGyroSlidesTurret(-5, 0.2, 3000, 0, 0, 0);


            /*
            drive.moveSlidesToHeightABS(2450, 0.4);
            drive.openClaw();
            sleep(200);

             */
            //Drop first cone in the junction

            /*drive.accelStraightGyroSlidesTurret(16.5, 0.5, 2450, 0, -440,0.3);
            drive.accelStraightGyroSlidesTurret(-3, 0.4, 2450, 0, -440,0);
            drive.moveSlidesToHeightABS(550, 0.8);
            sleep(200);
            drive.accelRightGyroSlidesTurret(26, 0.8, 550, 0, -440, 0.4, false);
            drive.accelStraightGyroSlidesTurret(0, 0, 550, 0, -440, 0.4);
            drive.coneLineDetect(true, false);
            drive.accelRightGyroSlidesTurret(3, 0.3, 530, 0, -440, 0, false);
            drive.closeClaw();
            sleep(200);
            //Pick up second cone from stack

            drive.moveSlidesToHeightABS(1100, 0.7);
            drive.turnTankGyro(-2, 0.2);
            drive.accelLeftGyroSlidesTurret(35, 0.8, 1150, 0.2, -880, 0.4, true);
            drive.strafeIMUJunction(true, 0.2, stageSwitchingPipeline);
            drive.accelStraightGyroSlidesTurret(-3, 0.2, 2500, 0.9, -880, 0);
            sleep(200);
            drive.moveSlidesToHeightABS(2400, 0.2);
            drive.openClaw();
            sleep(200);
            //Drop the second cone on the medium junction

            drive.accelStraightGyroSlidesTurret(1.5, 0.2, 2450, 0, -440, 0.4);
            drive.accelRightGyroSlidesTurret(35, 0.75, 600, 0.7, -440, 0.4, true);
            drive.accelStraightGyroSlidesTurret(0, 0, 530, 0, -440, 0.4);
            drive.coneLineDetect(true, false);
            sleep(200);
            drive.accelRightGyroSlidesTurret(2, 0.3, 500, 0, -440, 0, false);
            drive.closeClaw();
            sleep(200);
            //Pick up the third cone

            drive.moveSlidesToHeightABS(1100, 0.7);
            drive.turnTankGyro(-2.5, 0.2);
            drive.accelLeftGyroSlidesTurret(34, 0.8, 1100, 0.2, -880, 0.4, true);
            drive.strafeIMUJunction(true, 0.2, stageSwitchingPipeline);
            drive.accelStraightGyroSlidesTurret(-3, 0.2, 2500, 0.9, -880, 0);
            sleep(200);
            drive.moveSlidesToHeightABS(2400, 0.4);
            drive.openClaw();
            //Drop the third cone

            drive.accelStraightGyroSlidesTurret(2, 0.3, 2400, 0, -440, 0.3);
            drive.closeClaw();

*/
            //Move to the medium junction from the start and drop the cone

            if(tagOfInterest.id == LEFT)
                drive.accelLeftGyroSlidesTurret(14, 0.5, 200, 0.4, 0, 0.6);
                //Move to square one

            else if(tagOfInterest.id == MIDDLE)
                drive.accelRightGyroSlidesTurret(14, 0.5, 200, 0.4, 0, 0.6);
                //Move to square two

            else if(tagOfInterest.id == RIGHT) {
                drive.accelRightGyroSlidesTurret(40, 0.8, 200, 0.5, 0, 0.7);
                drive.accelStraightGyroSlidesTurret(-4, 0.3, 200, 0, 0, 0);
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
