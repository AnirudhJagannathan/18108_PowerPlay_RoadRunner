package org.firstinspires.ftc.teamcode.drive.opmode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.JUNCTION_MEDIUM;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.JUNCTION_LOW;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.JUNCTION_GROUND;

import android.graphics.drawable.VectorDrawable;
import android.telephony.mbms.MbmsErrors;

import java.security.acl.LastOwnerException;

@Autonomous
public class MyOpmode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(this, hardwareMap);

        while (opModeInInit()) {
            drive.closeClaw();
        }
        waitForStart();

        drive.coneLineDetect(true, true);

        if(isStopRequested()) return;

    }
}
