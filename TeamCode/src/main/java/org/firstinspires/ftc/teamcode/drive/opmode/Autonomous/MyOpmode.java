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

        Pose2d startPose = new Pose2d(-68.125, -36, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        /* Trajectory traj1 = drive.trajectoryBuilder(startPose, false)
                .lineToSplineHeading(new Pose2d(-25, -42, Math.toRadians(92)))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end(), false)
                .lineToConstantHeading(new Vector2d(-12, -42))
                .build();

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-25, -42, Math.toRadians(92)))
                .lineToConstantHeading(new Vector2d(-12, -42))
                .turn(Math.toRadians(183))
                .lineToSplineHeading(new Pose2d(-11, -55, Math.toRadians(-87)))
                .build();

         */

        /* Trajectory traj = drive.trajectoryBuilder(startPose, false)
                .lineToSplineHeading(new Pose2d(-23.5, -42, Math.toRadians(92)))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj.end(), false)
                .forward(5)
                .build();
         */

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-44, -42, Math.toRadians(93.5)))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end(), false)
                .forward(7.25)
                .build();

        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                .back(8.3)
                .waitSeconds(0.8)
                .strafeRight(3)
                .turn(Math.toRadians(-93))
                .lineToLinearHeading(new Pose2d(-16, -31, Math.toRadians(0)))
                .turn(Math.toRadians(-92))
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineToLinearHeading(new Pose2d(-7, -52.5, Math.toRadians(-90)))
                .build();

        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(traj4.end())
                .lineToConstantHeading(new Vector2d(-7, -30))
                .lineToConstantHeading(new Vector2d(-21, -30))
                .waitSeconds(0.3)
                .build();

        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .forward(3)
                .build();


        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(traj1);
        drive.moveSlidesToHeight(JUNCTION_LOW);
        drive.followTrajectory(traj2);
        drive.openClaw();
        drive.followTrajectorySequence(traj3);
        drive.moveSlidesToHeight(650);
        drive.followTrajectory(traj4);
        drive.closeClaw();
        drive.moveSlidesToHeight(JUNCTION_LOW + 100);
        drive.followTrajectorySequence(traj5);
        drive.followTrajectory(traj6);
        drive.openClaw();

    }
}
