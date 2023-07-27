package org.firstinspires.ftc.teamcode.drive.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class OdometryTest extends LinearOpMode {
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(this, hardwareMap);

        while (opModeInInit()) {
            for (int i = 0; i <= 3; i++) {
                drive.getMotors().get(i).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.getMotors().get(i).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            telemetry.addData("Left Wheel:", drive.getMotors().get(0).getCurrentPosition());
            telemetry.addData("Middle Wheel:", drive.getMotors().get(3).getCurrentPosition());
            telemetry.addData("Right Wheel:", drive.getMotors().get(1).getCurrentPosition());
            telemetry.update();
        }

        waitForStart();

        double avgDist = 0;
        while (avgDist < 10000) {
            avgDist = (drive.getMotors().get(0).getCurrentPosition() + drive.getMotors().get(1).getCurrentPosition()) / 2;
            drive.setMotorPowers(-0.3, -0.3, 0.3, 0.3);
        }
        drive.stop();
    }
}
