package org.firstinspires.ftc.teamcode.drive.opmode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
public class TeleOp_Drive extends LinearOpMode {

    private int HIGH_JUNCTION = 2500;
    private int MEDIUM_JUNCTION = 1500;
    private int LOW_JUNCTION = 750;
    private int GROUND_JUNCTION = 100;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(this, hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            /* drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
             */
            drive.mecanumDriving();
            drive.update();
            drive.moveSlides();
            drive.moveTurret();

            if (gamepad2.right_bumper)
                drive.openClaw();
            if (gamepad2.left_bumper)
                drive.closeClaw();
            /* if (gamepad2.a)
                drive.moveSlidesToHeight(HIGH_JUNCTION);
            if (gamepad2.b)
                drive.moveSlidesToHeight(MEDIUM_JUNCTION);
            if (gamepad2.y)
                drive.moveSlidesToHeight(LOW_JUNCTION);
            if (gamepad2.x)
                drive.moveSlidesToHeight(GROUND_JUNCTION);

             */

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
