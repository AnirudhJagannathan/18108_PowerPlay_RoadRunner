package org.firstinspires.ftc.teamcode.drive.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.JUNCTION_LOW;

import org.firstinspires.ftc.teamcode.drive.opmode.Autonomous.StageSwitchingPipeline;

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
public class RedLeftTest extends LinearOpMode {
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
        StageSwitchingPipeline stageSwitchingPipeline = new StageSwitchingPipeline();

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
        }

        else if (tagOfInterest != null) {
            drive.resetTurret();
            drive.resetSlides();

            drive.closeClaw();
            camera.setPipeline(stageSwitchingPipelineLeftSide);

            drive.accelStraightGyroSlidesTurret(38.5, 0.45, 1500, 0.8, 440,0.3);
            drive.accelLeftGyroSlidesTurret(2.5, 0.3,1500, 0.2, 440, 0.3);
            drive.moveSlidesToHeightABS(1425, 0.4);
            drive.openClaw();
            sleep(200);
            //Drop first cone in the junction

            drive.accelRightGyroSlidesTurret(2, 0.3, 1425, 0.4, 440, 0.3);
            drive.accelStraightGyroSlidesTurret(15, 0.5, 1425, 0, 440,0);
            drive.accelStraightGyroSlidesTurret(-3, 0.4, 1300, 0.4, 440,0);
            drive.accelLeftGyroSlidesTurret(21, 0.7, 550, 0.5, 440, 0);
            drive.accelStraightGyroSlidesTurret(0, 0, 550, 0, 440, 0.4);
            drive.accelStraightGyroSlidesTurret(0, 0, 500, 0, 440, 0);
            drive.coneLineDetect(true, true);
            drive.accelLeftGyroSlidesTurret(3.5, 0.3, 550, 0, 440, 0);
            drive.closeClaw();
            sleep(100);

            drive.accelRightGyroSlidesTurret(3.5, 0.25, 900, 0.6, 440, 0);
            drive.accelRightGyroSlidesTurret(4, 0.35, 150, 0.2, 880, 0.5);
            drive.strafeIMUJunctionLeft(false, 0.25, stageSwitchingPipelineLeftSide);
            drive.accelStraightGyroSlidesTurret(-5, 0.3, 1500, 0.8, 880, 0);
            drive.moveSlidesToHeightABS(1425, 0.4);
            drive.openClaw();
            sleep(100);

            drive.accelStraightGyroSlidesTurret(2.5, 0.3, 1425, 0, 440, 0.4);
            drive.accelLeftGyroSlidesTurret(8, 0.3, 500, 0.7, 440, 0.4);
            drive.accelStraightGyroSlidesTurret(0, 0, 500, 0, 440, 0);
            drive.coneLineDetect(true, true);
            drive.accelLeftGyroSlidesTurret(3, 0.3, 500, 0, 440, 0);
            drive.closeClaw();

            drive.accelRightGyroSlidesTurret(3.5, 0.25, 900, 0.6, 440, 0);
            drive.accelRightGyroSlidesTurret(4, 0.25, 150, 0.2, 880, 0.5);
            drive.strafeIMUJunctionLeft(false, 0.25, stageSwitchingPipelineLeftSide);
            drive.accelStraightGyroSlidesTurret(-5, 0.3, 1500, 0.8, 880, 0);
            drive.moveSlidesToHeightABS(1425, 0.4);
            drive.openClaw();
            sleep(100);

            drive.accelStraightGyroSlidesTurret(2, 0.3, 1425, 0, 440, 0.4);
            /* drive.accelLeftGyroSlidesTurret(10, 0.3, 500, 0.7, 440, 0.4, true);
            drive.coneLineDetect(true, true);
            drive.accelLeftGyroSlidesTurret(3, 0.3, 500, 0, 440, 0, false);
            drive.closeClaw();

            drive.accelRightGyroSlidesTurret(3, 0.25, 900, 0.6, 440, 0, true);
            drive.accelRightGyroSlidesTurret(6, 0.35, 150, 0.2, 880, 0.5, true);
            drive.strafeIMUJunctionLeft(false, 0.25, stageSwitchingPipelineLeftSide);
            drive.accelStraightGyroSlidesTurret(-4.5, 0.3, 1500, 0.8, 880, 0);
            drive.moveSlidesToHeightABS(1425, 0.4);
            drive.openClaw();
            sleep(100);

             */

            if(tagOfInterest.id == LEFT)
                drive.accelLeftGyroSlidesTurret(12, 0.5, 300, 0.6, 0, 0.6);
                //Move to square one

            else if(tagOfInterest.id == MIDDLE) {
                drive.turnTankGyro(-2.75, 0.3);
                drive.accelRightGyroSlidesTurret(14, 0.5, 300, 0.6, 0, 0.6);
            }
                //Move to square two

            else if(tagOfInterest.id == RIGHT) {
                drive.turnTankGyro(-2.75, 0.3);
                drive.accelRightGyroSlidesTurret(42, 0.8, 300, 0.4, 0, 0.7);
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
