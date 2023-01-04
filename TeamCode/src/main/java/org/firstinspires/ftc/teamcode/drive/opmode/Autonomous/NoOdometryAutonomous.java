package org.firstinspires.ftc.teamcode.drive.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.JUNCTION_LOW;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class NoOdometryAutonomous extends LinearOpMode {
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

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
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

        }

        else if (tagOfInterest != null) {
            drive.closeClaw();
            drive.moveSlidesToHeightABS(700);
            //Close the claw over the cone and raise slides up
            drive.driveStraightGyroSlidesTurret(16.2, 0.25, 2200, 0.8, 400, 0.7);
            //Drive to the small junction and rotate turret
            drive.openClaw();
            //Drop cone
            drive.strafeRightGyroSlidesTurret(1, 0.75, 2200,0.8, 400, 0.7);
            drive.driveStraightGyroSlidesTurret(1, 0.75, 2100, 0.8, -440, 1);
            drive.closeClaw();
            drive.driveStraightGyroSlidesTurret(36, 0.75, 2100, 0.8, -440, 1);
            sleep(200);
            drive.driveStraightGyroSlidesTurret(-3, 0.4, 750, 0.8, -440, 0.25);
            //Move forward and push signal sleeve out of the way
            drive.openClaw();
            drive.strafeRightGyroSlidesTurret(28, 0.7, 750, 0.8, -440, 0.25);
            drive.strafeRightGyroSlidesTurret(3, 0.3, 750, 0.8, -440, 0.25);
            drive.closeClaw();
            //Move to cone stack and grab cone
            sleep(200);
            drive.driveStraightGyroSlidesTurret(0, 0, 1500, 1, -440, 0.25);
            drive.strafeLeftGyroSlidesTurret(12.5, 0.6, 2050, 0.5, -880, 0.25);
            drive.driveStraightGyroSlidesTurret(-2.25, 0.3, 2050, 0.5, -880, 0.25);
            drive.openClaw();
            sleep(200);
            //Drop cone in the other low junction
            drive.driveStraightGyroSlidesTurret(2.25, 0.3, 2050, 0.5, -440, 0.25);
            drive.strafeRightGyroSlidesTurret(10, 0.7, 750, 0.8, -440, 0.25);
            drive.strafeRightGyroSlidesTurret(3.25, 0.3, 650, 0.8, -440, 0.25);
            drive.closeClaw();
            //Move to cone stack and grab cone
            sleep(200);
            drive.driveStraightGyroSlidesTurret(0, 0, 1500, 1, -440, 0.25);
            drive.strafeLeftGyroSlidesTurret(39.5, 0.85, 3650, 0.5, -880, 0.25);
            sleep(350);
            drive.driveStraightGyroSlidesTurret(-2, 0.4, 3650, 0.5, -880, 0.25);
            drive.openClaw();
            sleep(300);
            //Move to the medium junction and drop the cone
            drive.driveStraightGyroSlidesTurret(4, 0.3, 2100, 0.5, -440, 0.25);
            drive.strafeRightGyroSlidesTurret(36, 0.7, 500, 0.8, -440, 0.25);
            drive.strafeRightGyroSlidesTurret(4, 0.3, 400, 0.8, -440, 0.25);
            drive.closeClaw();
            sleep(200);
            //Grab another cone
            drive.driveStraightGyroSlidesTurret(0, 0, 1500, 1, -440, 0.25);
            drive.strafeLeftGyroSlidesTurret(38, 0.65, 4850, 0.75, 0, 0.25);
            sleep(400);
            drive.driveStraightGyroSlidesTurret(1.25, 0.2, 4850, 0.75, 0, 0.25);
            drive.openClaw();
            //Drop the cone in the high junction

            if(tagOfInterest.id == LEFT)
            {

            } //Move to square three

            else if(tagOfInterest.id == MIDDLE)
            {

            } //Move to square two

            else if(tagOfInterest.id == RIGHT)
            {

            } //Stay in that square
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
