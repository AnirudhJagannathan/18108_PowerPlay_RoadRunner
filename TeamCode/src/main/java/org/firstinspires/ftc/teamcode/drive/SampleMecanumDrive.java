package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import android.app.Activity;
import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.opmode.Autonomous.StageSwitchingPipeline;
import org.firstinspires.ftc.teamcode.drive.opmode.Autonomous.StageSwitchingPipelineLeftSide;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

import android.util.Log;
import android.view.View;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class SampleMecanumDrive extends MecanumDrive {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(9, 0, 1);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(10, 0, 1);

    public static double LATERAL_MULTIPLIER = 1.5388999714;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    // Constants define movement of claw; Min and Max Positions
    public static final double INCREMENT   = 0.05;     // amount to slew servo each CYCLE_MS cycle
    public static final int    CYCLE_MS    =   30;     // period of each cycle

    public static double AMAX_POS = 0.48;     // Maximum rotational position ---- Phil Claw: 1.4; GoBilda Claw: 1.4
    public static double AMIN_POS = 0.21;     // Minimum rotational position ---- Phil Claw: 0.7; GoBilda Claw: 0.61
    public double  Aposition = AMIN_POS;                 // Start position

    public static final double BMAX_POS     =  1.00;     // Maximum rotational position
    public static final double BMIN_POS     =  0.40;     // Minimum rotational position
    public double  Bposition = BMIN_POS;                 // Start position

    public static double CLAW_OPEN_SETTING = AMAX_POS;
    public static double CLAW_CLOSED_SETTING = AMIN_POS;

    Orientation lastAngles = new Orientation();
    double globalAngle, startAngle, endAngle, currentAngle;

    // Constants define junction height at tile intersections in units of linear slide motor encoder counts
    public static final int     JUNCTION_HIGH             = 2400;    // Height of junctions - highest
    public static final int     JUNCTION_MEDIUM           = 2500;    // Height of junctions - medium
    public static final int     JUNCTION_LOW              = 1450;     // Height of junctions - shortest
    public static final int     JUNCTION_GROUND           = 100;     // Height of junctions with no pole

    private TrajectoryFollower follower;

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFront, leftRear, rightRear, rightFront, slideLeft, slideRight, turret;
    private Servo Claw;
    private List<DcMotorEx> motors;

    public ColorSensor colorSensor;
    public NormalizedRGBA colors;

    private BNO055IMU imu;
    public BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    private VoltageSensor batteryVoltageSensor;
    private LinearOpMode opMode;

    public double  targetHeading = 0;
    public double  driveSpeed    = 0;
    public double  turnSpeed     = 0;
    public double  leftSpeed     = 0;
    public double  rightSpeed    = 0;
    public double leftPower = 0.0;
    public double rightPower = 0.0;
    public double drive1 = 0.0;
    public double drive2 = 0.0;
    public double turn1 = 0.0;
    public double turn2 = 0.0;
    public int     leftFrontTarget  = 0;
    public int     rightFrontTarget = 0;
    public int     leftRearTarget   = 0;
    public int     rightRearTarget  = 0;
    public double startMotorCounts = 0;
    public double stopMotorCounts = 0;

    public boolean aCurrentState = false;
    public boolean xCurrentState = false;
    public boolean yCurrentState = false;
    public boolean bCurrentState = false;

    public boolean aLastState = false;
    public boolean xLastState = false;
    public boolean yLastState = false;
    public boolean bLastState = false;

    static final double     STRAFE_COUNTS_PER_INCH  = 45.178235; //47.4369230781 non acceleration
    static final double     WHEEL_COUNTS_PER_INCH   = 142.3636363625;

    public SampleMecanumDrive(LinearOpMode opMode, HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);
        this.opMode = opMode;

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: If the hub containing the IMU you are using is mounted so that the "REV" logo does
        // not face up, remap the IMU axes so that the z-axis points upward (normal to the floor.)
        //
        //             | +Z axis
        //             |
        //             |
        //             |
        //      _______|_____________     +Y axis
        //     /       |_____________/|__________
        //    /   REV / EXPANSION   //
        //   /       / HUB         //
        //  /_______/_____________//
        // |_______/_____________|/
        //        /
        //       / +X axis
        //
        // This diagram is derived from the axes in section 3.4 https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf
        // and the placement of the dot/orientation from https://docs.revrobotics.com/rev-control-system/control-system-overview/dimensions#imu-location
        //
        // For example, if +Y in this diagram faces downwards, you would use AxisDirection.NEG_Y.
        // BNO055IMUUtil.remapZAxis(imu, AxisDirection.NEG_Y);

        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        leftRear = hardwareMap.get(DcMotorEx.class, "backLeft");
        rightRear = hardwareMap.get(DcMotorEx.class, "backRight");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        slideLeft = hardwareMap.get(DcMotorEx.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotorEx.class, "slideRight");
        turret = hardwareMap.get(DcMotorEx.class, "turret");

        Claw = hardwareMap.get(Servo.class, "Claw");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Initialize values for IMU
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()
        //leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: if desired, use setLocalizer() to change the localization method
        setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));
        setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));

        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }


    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES);
        globalAngle = 0;
    }

    private double getAngle() {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        opMode.telemetry.addData("Gyro Angle", "(%.2f)", angles.firstAngle);
        opMode.telemetry.addData("Encoders:", "M0: %3d  M1:%3d  M2:%3d  M3:%3d",
                leftFront.getCurrentPosition(),
                rightFront.getCurrentPosition(),
                leftRear.getCurrentPosition(),
                rightRear.getCurrentPosition());
        opMode.telemetry.update();

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;
        lastAngles = angles;

        return globalAngle;
    }

    private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .02;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }


    public void resetSlides() {
        slideLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        slideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetTurret() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
        slideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
        slideLeft.setZeroPowerBehavior(zeroPowerBehavior);
        slideRight.setZeroPowerBehavior(zeroPowerBehavior);
        turret.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }                                                                   

        setDrivePower(vel);
    }

    public void moveSlides() {
        double slidePower = Range.clip(opMode.gamepad2.right_stick_y, -1.0, 1.0);

        //Limit how high the slides can go

        if(((slideLeft.getCurrentPosition()) > 3650 && slidePower < 0) ||
                ((slideLeft.getCurrentPosition()) < -150 && slidePower > 0)) {
            slideLeft.setPower(0);
            slideRight.setPower(0);
        }
        else if (slidePower > 0) {
            slideLeft.setPower(slidePower);
            slideRight.setPower(slidePower);
        }
        else {
            slideLeft.setPower(slidePower);
            slideRight.setPower(slidePower);
        }

        opMode.telemetry.addData("slidePower", slidePower);
        opMode.telemetry.addData("slideLeftHeight", slideLeft.getCurrentPosition());
        opMode.telemetry.addData("slideRightHeight", slideRight.getCurrentPosition());
    }


        /* if ((-slideLeft.getCurrentPosition() + slideRight.getCurrentPosition())/2 > 4000) {
            slideLeft.setPower(0.0);
            slideLeft.setPower(0.0);
        }

         */

    public void moveTurret() throws InterruptedException {
        double turretPower = Range.clip(opMode.gamepad2.left_stick_x, -1.0, 1.0);
        double turretPos = turret.getCurrentPosition();
        opMode.telemetry.addData("turretPos", turretPos);

        if ((turretPos >= 850 && turretPower < -0.01) || (turretPos <= -850 && turretPower > 0.01)) {
            turret.setPower(0);
        }
        else {
            turret.setPower(-0.6 * turretPower);
        }
    }

    public void moveTurretToPosition(int motorTurretEncoderCounts) {
        int turretHalfRotationCounts = 880;  // This number is just a guess, need to check on B Bot

        turret.setTargetPosition(motorTurretEncoderCounts);
        turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        if (Math.abs(motorTurretEncoderCounts) < turretHalfRotationCounts) {
            if (turret.getCurrentPosition() < motorTurretEncoderCounts) {
                turret.setPower(0.50);
            } else {
                turret.setPower(-0.50);
            }
        }

        else if (turret.getCurrentPosition() > 0 && motorTurretEncoderCounts == 880)
            turret.setPower(0.50);
        else if (turret.getCurrentPosition() < 0 && motorTurretEncoderCounts == 880) {
            turret.setTargetPosition(-motorTurretEncoderCounts);
            turret.setPower(-0.50);
        }
    }

    public void moveTurretToPositionTeleOP() {
        aLastState = aCurrentState;
        aCurrentState = opMode.gamepad2.a;
        xLastState = xCurrentState;
        xCurrentState = opMode.gamepad2.x;
        yLastState = yCurrentState;
        yCurrentState = opMode.gamepad2.y;
        bLastState = bCurrentState;
        bCurrentState = opMode.gamepad2.b;

        if (aCurrentState && !aLastState)
            moveTurretToPosition(880);
        if (xCurrentState && !xLastState)
            moveTurretToPosition(440);
        if (bCurrentState && !bLastState)
            moveTurretToPosition(-440);
        if (yCurrentState && !yLastState)
            moveTurretToPosition(0);
    }

    public void openClaw() {
        Claw.setPosition(AMAX_POS);
    }

    public void closeClaw() {
        Claw.setPosition(CLAW_CLOSED_SETTING);
    }

    public void mecanumDriving() {
        double drive = opMode.gamepad1.left_stick_y;
        double strafe = opMode.gamepad1.right_stick_x;
        double turn = opMode.gamepad1.left_stick_x;
        double v1, v2, v3, v4;

        if (opMode.gamepad1.right_bumper) {
            v1 = Range.clip(-drive + strafe + turn, -0.2, 0.2);
            v2 = Range.clip(-drive - strafe - turn, -0.2, 0.2);
            v3 = Range.clip(-drive + strafe - turn, -0.2, 0.2);
            v4 = Range.clip(-drive - strafe + turn, -0.2, 0.2);
        }

        else {
            v1 = Range.clip(-drive + strafe + turn, -0.85, 0.85);
            v2 = Range.clip(-drive - strafe - turn, -0.85, 0.85);
            v3 = Range.clip(-drive + strafe - turn, -0.85, 0.85);
            v4 = Range.clip(-drive - strafe + turn, -0.85, 0.85);
        }
        leftFront.setPower(v1);
        rightFront.setPower(v2);
        leftRear.setPower(v3);
        rightRear.setPower(v4);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    public void strafeIMUJunction(boolean leftOrRight,
                                  double maxDriveSpeed, StageSwitchingPipeline pipeline) { //True = left, False = right

        int valMid;
        double correction;

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            drive1 = 0.0;
            if (leftOrRight){
                drive2 = maxDriveSpeed;
            }
            else {
                drive2 = -maxDriveSpeed;
            }
            leftPower = Range.clip(drive1 + drive2, -1.0, 1.0);
            rightPower = Range.clip(drive1 - drive2, -1.0, 1.0);
            // Send calculated power to wheels
            leftFront.setPower(rightPower);
            rightFront.setPower(leftPower);
            leftRear.setPower(leftPower);
            rightRear.setPower(rightPower);

            leftFront.setPower(leftPower);
            rightFront.setPower(rightPower);
            leftRear.setPower(rightPower);
            rightRear.setPower(leftPower);

            while (true) {
                correction = checkDirection();
                valMid = pipeline.getValMid();

                leftFront.setPower(rightPower - correction);
                rightFront.setPower(leftPower + correction);
                leftRear.setPower(leftPower - correction);
                rightRear.setPower(rightPower + correction);


                if (valMid > 100) {
                    stop();
                    break;
                }

                opMode.telemetry.addData("valMid",  pipeline.getValMid());
                opMode.telemetry.addData("Height", pipeline.getRows());
                opMode.telemetry.addData("Width", pipeline.getCols());

                opMode.telemetry.update();
            }
        }
    }

    public void moveIMUJunction(double maxDriveSpeed, StageSwitchingPipeline pipeline) { //True = left, False = right

        int valMid;
        double correction;

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        resetAngle();

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            leftFront.setPower(maxDriveSpeed);
            rightFront.setPower(maxDriveSpeed);
            leftRear.setPower(maxDriveSpeed);
            rightRear.setPower(maxDriveSpeed);

            while (true) {
                correction = checkDirection();
                valMid = pipeline.getValMid();

                leftFront.setPower(maxDriveSpeed - correction);
                rightFront.setPower(maxDriveSpeed + correction);
                leftRear.setPower(maxDriveSpeed - correction);
                rightRear.setPower(maxDriveSpeed + correction);


                if (valMid > 100) {
                    stop();
                    break;
                }

                opMode.telemetry.addData("valMid",  pipeline.getValMid());
                opMode.telemetry.addData("Height", pipeline.getRows());
                opMode.telemetry.addData("Width", pipeline.getCols());

                opMode.telemetry.update();
            }
        }
    }

    public void strafeIMUJunctionLeft(boolean leftOrRight,
                                  double maxDriveSpeed, StageSwitchingPipelineLeftSide pipeline) { //True = left, False = right

        int valMid;
        double correction;

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        resetAngle();

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            drive1 = 0.0;
            if (leftOrRight){
                drive2 = maxDriveSpeed;
            }
            else {
                drive2 = -maxDriveSpeed;
            }
            leftPower = Range.clip(drive1 + drive2, -1.0, 1.0);
            rightPower = Range.clip(drive1 - drive2, -1.0, 1.0);
            // Send calculated power to wheels
            leftFront.setPower(rightPower);
            rightFront.setPower(leftPower);
            leftRear.setPower(leftPower);
            rightRear.setPower(rightPower);

            leftFront.setPower(leftPower);
            rightFront.setPower(rightPower);
            leftRear.setPower(rightPower);
            rightRear.setPower(leftPower);

            while (true) {
                correction = checkDirection();
                valMid = pipeline.getValMid();

                leftFront.setPower(rightPower - correction);
                rightFront.setPower(leftPower + correction);
                leftRear.setPower(leftPower - correction);
                rightRear.setPower(rightPower + correction);


                if (valMid > 100) {
                    stop();
                    break;
                }

                opMode.telemetry.addData("valMid",  pipeline.getValMid());
                opMode.telemetry.addData("Height", pipeline.getRows());
                opMode.telemetry.addData("Width", pipeline.getCols());

                opMode.telemetry.update();
            }
        }
    }

    public void turnTankGyro(double angleToTurn, double anglePower) {
        double angle = angleToTurn;
        double power = anglePower;

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        opMode.telemetry.addData("Gyro Angle", "(%.2f)", angles.firstAngle);
        opMode.telemetry.addData("Encoders:", "M0: %3d  M1:%3d  M2:%3d  M3:%3d",
                leftFront.getCurrentPosition(),
                rightFront.getCurrentPosition(),
                leftRear.getCurrentPosition(),
                rightRear.getCurrentPosition());
        opMode.telemetry.update();

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        resetAngle();

        currentAngle = getAngle();
        startAngle = currentAngle;

        opMode.sleep(150);

        if (angle <= 0) {
            // Start Right turn
            leftFront.setPower(power);
            rightFront.setPower(-1 * power);
            leftRear.setPower(power);
            rightRear.setPower(-1 * power);

            while (true) {
                currentAngle = getAngle();

                if (currentAngle <= 0.5 * angle) {
                    leftFront.setPower(0.9 * power);
                    rightFront.setPower(-0.9 * power);
                    leftRear.setPower(0.9 * power);
                    rightRear.setPower(-0.9 * power);
                }

                // Stop turning when the turned angle = requested angle
                if (currentAngle <= angle) {
                    leftFront.setPower(0.0);
                    rightFront.setPower(0.0);
                    leftRear.setPower(0.0);
                    rightRear.setPower(0.0);
                    break;
                }
                opMode.telemetry.addData("Gyro Angle", "(%.2f)", angles.firstAngle);
                opMode.telemetry.addData("Encoders:", "M0: %3d  M1:%3d  M2:%3d  M3:%3d",
                        leftFront.getCurrentPosition(),
                        rightFront.getCurrentPosition(),
                        leftRear.getCurrentPosition(),
                        rightRear.getCurrentPosition());
                opMode.telemetry.update();
            }
        } else {
            // Start Left turn
            leftFront.setPower(-1 * power);
            rightFront.setPower(power);
            leftRear.setPower(-1 * power);
            rightRear.setPower(power);

            while (true) {
                currentAngle = getAngle();

                if (currentAngle >= 0.5 * angle) {
                    leftFront.setPower(-0.9 * power);
                    rightFront.setPower(0.9 * power);
                    leftRear.setPower(-0.9 * power);
                    rightRear.setPower(0.9 * power);
                }

                // Stop turning when the turned angle = requested angle
                if (currentAngle >= angle) {
                    leftFront.setPower(0.0);
                    rightFront.setPower(0.0);
                    leftRear.setPower(0.0);
                    rightRear.setPower(0.0);
                    break;
                }
                opMode.telemetry.addData("Gyro Angle", "(%.2f)", angles.firstAngle);
                opMode.telemetry.addData("Encoders:", "M0: %3d  M1:%3d  M2:%3d  M3:%3d",
                        leftFront.getCurrentPosition(),
                        rightFront.getCurrentPosition(),
                        leftRear.getCurrentPosition(),
                        rightRear.getCurrentPosition());
                opMode.telemetry.update();
            }
        }
        resetAngle();
    }

    public void driveStraightGyro(double inchesToDrive, double drivePower) {

        double power = drivePower;
        double motorDistance = (double) (inchesToDrive * WHEEL_COUNTS_PER_INCH);
        double correction = 0.02;

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetAngle();

        opMode.sleep(250);

        if (motorDistance >= 0) {
            // Start driving forward
            leftFront.setPower(0.25);
            rightFront.setPower(0.25);
            leftRear.setPower(0.25);
            rightRear.setPower(0.25);

            while (true) {
                // telemetry.addData("leftFront",  "Distance: %3d", leftFront.getCurrentPosition());
                // telemetry.update();

                correction = checkDirection();
                leftFront.setPower(power - correction);
                rightFront.setPower(power + correction);
                leftRear.setPower(power - correction);
                rightRear.setPower(power + correction);

                Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                opMode.telemetry.addData("Gyro Angle", "(%.2f)", angles.firstAngle);
                opMode.telemetry.addData("Encoders:", "M0: %3d  M1:%3d  M2:%3d  M3:%3d",
                        leftFront.getCurrentPosition(),
                        -rightFront.getCurrentPosition(),
                        -leftRear.getCurrentPosition(),
                        rightRear.getCurrentPosition());
                opMode.telemetry.update();

                // Stop driving when Motor Encoder Avg. >= motorDistance
                if ((leftFront.getCurrentPosition() - rightFront.getCurrentPosition() - leftRear.getCurrentPosition() + rightRear.getCurrentPosition()) / 4.0 >= motorDistance) {
                    leftFront.setPower(0.0);
                    rightFront.setPower(0.0);
                    leftRear.setPower(0.0);
                    rightRear.setPower(0.0);
                    break;
                }
            }
        }
        if (motorDistance < 0) {
            // Start driving backward
            leftFront.setPower(-power);
            rightFront.setPower(-power);
            leftRear.setPower(-power);
            rightRear.setPower(-power);

            while (true) {
                // telemetry.addData("leftFront",  "Distance: %3d", leftFront.getCurrentPosition());
                // telemetry.update();

                correction = checkDirection();
                leftFront.setPower(-1 * (power + correction));
                rightFront.setPower(-1 * (power - correction));
                leftRear.setPower(-1 * (power + correction));
                rightRear.setPower(-1 * (power - correction));

                Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                opMode.telemetry.addData("Gyro Angle", "(%.2f)", angles.firstAngle);
                opMode.telemetry.addData("Encoders:", "M0: %3d  M1:%3d  M2:%3d  M3:%3d",
                        leftFront.getCurrentPosition(),
                        rightFront.getCurrentPosition(),
                        leftRear.getCurrentPosition(),
                        rightRear.getCurrentPosition());
                opMode.telemetry.update();

                // Stop driving when Motor Encoder Avg. <= motorDistance
                if ((Math.abs(leftFront.getCurrentPosition()) + Math.abs(rightFront.getCurrentPosition()) +
                        Math.abs(leftRear.getCurrentPosition()) + Math.abs(rightRear.getCurrentPosition())) / -4.0
                        <= motorDistance) {
                    leftFront.setPower(0.0);
                    rightFront.setPower(0.0);
                    leftRear.setPower(0.0);
                    rightRear.setPower(0.0);
                    break;
                }
            }
        }
    }

    public void strafeLeftIMU(double distance,
                              double maxDriveSpeed) {

        double correction;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            opMode.sleep(500);

            startMotorCounts = leftFront.getCurrentPosition();
            stopMotorCounts = startMotorCounts - (int) (distance * STRAFE_COUNTS_PER_INCH);
            drive1 = 0.0;
            drive2 = maxDriveSpeed;
            leftPower = Range.clip(drive1 + drive2, -1.0, 1.0);
            rightPower = Range.clip(drive1 - drive2, -1.0, 1.0);
            // Send calculated power to wheels
            leftFront.setPower(rightPower);
            rightFront.setPower(leftPower);
            leftRear.setPower(leftPower);
            rightRear.setPower(rightPower);

            while (true) {

                correction = checkDirection();
                leftFront.setPower(rightPower - correction);
                rightFront.setPower(leftPower + correction);
                leftRear.setPower(leftPower - correction);
                rightRear.setPower(rightPower + correction);

                Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                opMode.telemetry.addData("Gyro Angle", "(%.2f)", angles.firstAngle);
                opMode.telemetry.addData("Encoders:", "M0: %3d  M1:%3d  M2:%3d  M3:%3d",
                        leftFront.getCurrentPosition(),
                        -rightFront.getCurrentPosition(),
                        -leftRear.getCurrentPosition(),
                        rightRear.getCurrentPosition());
                opMode.telemetry.update();

                // Stop when the bot has driven the requested distance
                if (leftFront.getCurrentPosition() <= stopMotorCounts) {
                    stop();
                    break;
                }
            }
        }
    }

    public void strafeRightIMU(double distance, double maxDriveSpeed) {
        double correction;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            opMode.sleep(500);

            startMotorCounts = leftFront.getCurrentPosition();
            stopMotorCounts = startMotorCounts + (int) (distance * STRAFE_COUNTS_PER_INCH);
            drive1 = 0.0;
            drive2 = -maxDriveSpeed;
            leftPower = Range.clip(drive1 + drive2, -1.0, 1.0);
            rightPower = Range.clip(drive1 - drive2, -1.0, 1.0);
            // Send calculated power to wheels
            leftFront.setPower(rightPower);
            rightFront.setPower(leftPower);
            leftRear.setPower(leftPower);
            rightRear.setPower(rightPower);

            while (true) {

                correction = checkDirection();
                leftFront.setPower(rightPower - correction);
                rightFront.setPower(leftPower + correction);
                leftRear.setPower(leftPower - correction);
                rightRear.setPower(rightPower + correction);

                Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                opMode.telemetry.addData("Gyro Angle", "(%.2f)", angles.firstAngle);
                opMode.telemetry.addData("Encoders:", "M0: %3d  M1:%3d  M2:%3d  M3:%3d",
                        leftFront.getCurrentPosition(),
                        -rightFront.getCurrentPosition(),
                        -leftRear.getCurrentPosition(),
                        rightRear.getCurrentPosition());
                opMode.telemetry.update();

                // Stop when the bot has driven the requested distance
                if (leftFront.getCurrentPosition() >= stopMotorCounts) {
                    stop();
                    break;
                }
            }
        }
    }

    public void moveSlidesAndTurret(double slideHeight, double slidePower, int turretPos, double turretSpeed) {

        slideLeft.setTargetPosition((int) slideHeight);
        slideRight.setTargetPosition((int) slideHeight);
        turret.setTargetPosition(turretPos);

        //slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // move slides
        if (slideLeft.getCurrentPosition() < slideHeight) {
            slideLeft.setPower(-slidePower);
            slideRight.setPower(-slidePower);
        }
        else if (slideLeft.getCurrentPosition() > slideHeight) {
            slideLeft.setPower(slidePower);
            slideRight.setPower(slidePower);
        }

        //move turret
        if (turret.getCurrentPosition() > turretPos)
            turret.setPower(-turretSpeed);
        else if (turret.getCurrentPosition() < turretPos)
            turret.setPower(turretSpeed);

        while (slideLeft.getCurrentPosition() < slideHeight - 10 || slideLeft.getCurrentPosition() > slideHeight + 10) {
            opMode.telemetry.addData("Encoders:", "M0: %3d  M1:%3d  M2:%3d  M3:%3d",
                    leftFront.getCurrentPosition(),
                    rightFront.getCurrentPosition(),
                    leftRear.getCurrentPosition(),
                    rightRear.getCurrentPosition());
            opMode.telemetry.update();
        }

        slideLeft.setPower(0);
        slideRight.setPower(0);
        turret.setPower(0);
    }

    public void driveStraightGyroSlidesTurret(double inchesToDrive, double drivePower, double slideHeight, double slidePower, int turretPos, double turretSpeed) {

        double power = drivePower;
        double motorDistance = (double) (inchesToDrive * WHEEL_COUNTS_PER_INCH);
        double correction = 0.02;
        boolean up = false;
        boolean down = false;

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideLeft.setTargetPosition((int) slideHeight);
        slideRight.setTargetPosition((int) slideHeight);
        turret.setTargetPosition(turretPos);

        //slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // move slides
        if (slideLeft.getCurrentPosition() < slideHeight) {
            slideLeft.setPower(-slidePower);
            slideRight.setPower(-slidePower);
            up = true;
        }
        else if (slideLeft.getCurrentPosition() > slideHeight) {
            slideLeft.setPower(slidePower);
            slideRight.setPower(slidePower);
            down = true;
        }

        //move turret
        if (turret.getCurrentPosition() > turretPos)
            turret.setPower(-turretSpeed);
        else if (turret.getCurrentPosition() < turretPos)
            turret.setPower(turretSpeed);

        if (motorDistance >= 0) {
            while (true) {
                // telemetry.addData("leftFront",  "Distance: %3d", leftFront.getCurrentPosition());
                // telemetry.update();

                correction = checkDirection();
                leftFront.setPower(power - correction);
                rightFront.setPower(power + correction);
                leftRear.setPower(power - correction);
                rightRear.setPower(power + correction);

                //slideLeft.setPower(slidePower);
                //slideRight.setPower(-slidePower);

                Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                opMode.telemetry.addData("Gyro Angle", "(%.2f)", angles.firstAngle);
                opMode.telemetry.addData("Encoders:", "M0: %3d  M1:%3d  M2:%3d  M3:%3d",
                        leftFront.getCurrentPosition(),
                        -rightFront.getCurrentPosition(),
                        -leftRear.getCurrentPosition(),
                        rightRear.getCurrentPosition());
                opMode.telemetry.addData("slideHeight", slideLeft.getCurrentPosition());
                opMode.telemetry.addData("slidePower", slidePower);
                opMode.telemetry.update();

                if (up && (double) (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2  >= slideHeight) {
                    slideLeft.setPower(0.0);
                    slideRight.setPower(0.0);
                }
                if (down && (double) (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2  <= slideHeight) {
                    slideLeft.setPower(0.0);
                    slideRight.setPower(0.0);
                }

                // Stop driving when Motor Encoder Avg. >= motorDistance
                if ((Math.abs(leftFront.getCurrentPosition()) + Math.abs(rightFront.getCurrentPosition()) +
                        Math.abs(leftRear.getCurrentPosition()) + Math.abs(rightRear.getCurrentPosition()) / 4.0
                        >= motorDistance)) {
                    leftFront.setPower(0.0);
                    rightFront.setPower(0.0);
                    leftRear.setPower(0.0);
                    rightRear.setPower(0.0);
                    break;
                }
            }
        }
        if (motorDistance < 0) {
            while (true) {
                // telemetry.addData("leftFront",  "Distance: %3d", leftFront.getCurrentPosition());
                // telemetry.update();

                correction = checkDirection();
                leftFront.setPower(-1 * (power + correction));
                rightFront.setPower(-1 * (power - correction));
                leftRear.setPower(-1 * (power + correction));
                rightRear.setPower(-1 * (power - correction));

                Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                opMode.telemetry.addData("Gyro Angle", "(%.2f)", angles.firstAngle);
                opMode.telemetry.addData("Encoders:", "M0: %3d  M1:%3d  M2:%3d  M3:%3d",
                        leftFront.getCurrentPosition(),
                        rightFront.getCurrentPosition(),
                        leftRear.getCurrentPosition(),
                        rightRear.getCurrentPosition());
                opMode.telemetry.update();

                if (up && (double) (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2  >= slideHeight) {
                    slideLeft.setPower(0.0);
                    slideRight.setPower(0.0);
                }
                if (down && (double) (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2  <= slideHeight) {
                    slideLeft.setPower(0.0);
                    slideRight.setPower(0.0);
                }

                // Stop driving when Motor Encoder Avg. <= motorDistance
                if ((Math.abs(leftFront.getCurrentPosition()) + Math.abs(rightFront.getCurrentPosition()) +
                        Math.abs(leftRear.getCurrentPosition()) + Math.abs(rightRear.getCurrentPosition()) / 4.0
                        >= Math.abs(motorDistance))) {
                    leftFront.setPower(0.0);
                    rightFront.setPower(0.0);
                    leftRear.setPower(0.0);
                    rightRear.setPower(0.0);
                    break;
                }
            }
        }
        if (slidePower != 0) {
            while (true) {
                if (up && (double) (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2 >= slideHeight) {
                    slideLeft.setPower(0.0);
                    slideRight.setPower(0.0);
                    break;
                }
                if (down && (double) (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2 <= slideHeight) {
                    slideLeft.setPower(0.0);
                    slideRight.setPower(0.0);
                    break;
                }
                opMode.telemetry.addData("slidePos", "position: %3d", slideLeft.getCurrentPosition());
                opMode.telemetry.addData("turretPos", "position: %3d", turret.getCurrentPosition());
                opMode.telemetry.update();
            }
        }

        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideLeft.setPower(0);
        slideRight.setPower(0);
        turret.setPower(0);
    }

    public void moveSlidesToHeightABS(int encoderPos, double power) {
        slideLeft.setTargetPosition(encoderPos);
        slideRight.setTargetPosition(encoderPos);

        while (slideLeft.getCurrentPosition() < encoderPos - 10 || slideLeft.getCurrentPosition() > encoderPos + 10) {
            if (slideLeft.getCurrentPosition() < encoderPos) {
                slideLeft.setPower(-power);
                slideRight.setPower(-power);
            } else {
                slideLeft.setPower(power);
                slideRight.setPower(power);
            }
        }

        slideLeft.setPower(0);
        slideRight.setPower(0);
    }

    public void accelStraightGyroSlidesTurret(double inchesToDrive, double drivePower, double slideHeight, double slidePower, int turretPos, double turretSpeed) {

        double power = drivePower;
        double motorDistance = (double) (inchesToDrive * WHEEL_COUNTS_PER_INCH);
        double correction = 0.02;
        boolean state1 = true;
        boolean state2 = true;
        boolean up = false;
        boolean down = false;
        boolean left = false;
        boolean right = false;


        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideLeft.setTargetPosition((int) slideHeight);
        slideRight.setTargetPosition((int) slideHeight);
        turret.setTargetPosition(turretPos);

        //slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        opMode.sleep(150);

        // move slides
        if (slideLeft.getCurrentPosition() < slideHeight) {
            slideLeft.setPower(-slidePower);
            slideRight.setPower(-slidePower);
            up = true;
        } else {
            slideLeft.setPower(slidePower);
            slideRight.setPower(slidePower);
            down = true;
        }

        //move turret
        if (turret.getCurrentPosition() > turretPos) {
            turret.setPower(-turretSpeed);
        }
        else if (turret.getCurrentPosition() < turretPos) {
            turret.setPower(turretSpeed);
        }

        if (motorDistance > 0) {
            while (true) {
                // telemetry.addData("leftFront",  "Distance: %3d", leftFront.getCurrentPosition());
                // telemetry.update();

                if ((Math.abs(leftFront.getCurrentPosition()) + Math.abs(rightFront.getCurrentPosition()) +
                        Math.abs(leftRear.getCurrentPosition()) + Math.abs(rightRear.getCurrentPosition())) / 4.0 < (motorDistance / 6) && state1)
                {
                    for (double i = 0; i <= 1; i += .0625) {
                        correction = checkDirection();
                        leftFront.setPower(i * (power - correction));
                        rightFront.setPower(i * (power + correction));
                        leftRear.setPower(i * (power - correction));
                        rightRear.setPower(i * (power + correction));
                        state1 = false;
                        if (up && (double) (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2  >= slideHeight) {
                            slideLeft.setPower(0.0);
                            slideRight.setPower(0.0);
                        }
                        if (down && (double) (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2  <= slideHeight) {
                            slideLeft.setPower(0.0);
                            slideRight.setPower(0.0);
                        }
                        /* if ((double) turret.getCurrentPosition() >= turretPos)
                            turret.setPower(0.0);
                        if ((double) turret.getCurrentPosition() <= turretPos)
                            turret.setPower(0.0);
                         */
                    }
                }

                correction = checkDirection();
                leftFront.setPower(power - correction);
                rightFront.setPower(power + correction);
                leftRear.setPower(power - correction);
                rightRear.setPower(power + correction);

                if (up && (double) (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2  >= slideHeight) {
                    slideLeft.setPower(0.0);
                    slideRight.setPower(0.0);
                }
                if (down && (double) (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2  <= slideHeight) {
                    slideLeft.setPower(0.0);
                    slideRight.setPower(0.0);
                }
                /* if (left && (double) turret.getCurrentPosition() >= turretPos)
                    turret.setPower(0.0);
                if (right && (double) turret.getCurrentPosition() <= turretPos)
                    turret.setPower(0.0);
                 */

                Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                opMode.telemetry.addData("Gyro Angle", "(%.2f)", angles.firstAngle);
                opMode.telemetry.addData("Encoders:", "M0: %3d  M1:%3d  M2:%3d  M3:%3d",
                        leftFront.getCurrentPosition(),
                        -rightFront.getCurrentPosition(),
                        -leftRear.getCurrentPosition(),
                        rightRear.getCurrentPosition());
                opMode.telemetry.addData("slideHeight", (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2);
                opMode.telemetry.addData("slidePower", slidePower);
                opMode.telemetry.update();

                if ((Math.abs(leftFront.getCurrentPosition()) + Math.abs(rightFront.getCurrentPosition()) +
                        Math.abs(leftRear.getCurrentPosition()) + Math.abs(rightRear.getCurrentPosition())) / 4.0 > ((4.0/6) * Math.abs(motorDistance)) && state2)
                {
                    opMode.telemetry.addData("turretPos", turret.getCurrentPosition());
                    opMode.telemetry.update();
                    for (double j = 1; j >= 0; j -= .0625) {
                        correction = checkDirection();
                        leftFront.setPower(j * (power - correction));
                        rightFront.setPower(j * (power + correction));
                        leftRear.setPower(j * (power - correction));
                        rightRear.setPower(j * (power + correction));
                        state2 = false;
                        if (up && (double) (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2  >= slideHeight) {
                            slideLeft.setPower(0.0);
                            slideRight.setPower(0.0);
                        }
                        if (down && (double) (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2  <= slideHeight) {
                            slideLeft.setPower(0.0);
                            slideRight.setPower(0.0);
                        }
                        /* if ((double) turret.getCurrentPosition() >= turretPos)
                            turret.setPower(0.0);
                        if ((double) turret.getCurrentPosition() <= turretPos)
                            turret.setPower(0.0);
                         */
                    }
                }

                // Stop driving when Motor Encoder Avg. >= motorDistance
                if ((Math.abs(leftFront.getCurrentPosition()) + Math.abs(rightFront.getCurrentPosition()) +
                        Math.abs(leftRear.getCurrentPosition()) + Math.abs(rightRear.getCurrentPosition()) / 4.0
                        >= motorDistance)) {
                    leftFront.setPower(0.0);
                    rightFront.setPower(0.0);
                    leftRear.setPower(0.0);
                    rightRear.setPower(0.0);
                    break;
                }
            }
        }

        if (motorDistance < 0) {
            while (true) {
                // telemetry.addData("leftFront",  "Distance: %3d", leftFront.getCurrentPosition());
                // telemetry.update();

                if ((Math.abs(leftFront.getCurrentPosition()) + Math.abs(rightFront.getCurrentPosition()) +
                        Math.abs(leftRear.getCurrentPosition()) + Math.abs(rightRear.getCurrentPosition())) / 4.0 < (Math.abs(motorDistance) / 6) && state1)
                {
                    for (double i = 0; i <= 1; i += .0625) {
                        correction = checkDirection();
                        leftFront.setPower(-i * (power + correction));
                        rightFront.setPower(-i * (power - correction));
                        leftRear.setPower(-i * (power + correction));
                        rightRear.setPower(-i * (power - correction));
                        state1 = false;
                        if (up && (double) (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2  >= slideHeight) {
                            slideLeft.setPower(0.0);
                            slideRight.setPower(0.0);
                        }
                        if (down && (double) (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2  <= slideHeight) {
                            slideLeft.setPower(0.0);
                            slideRight.setPower(0.0);
                        }
                        /* if (left && (double) turret.getCurrentPosition() >= turretPos)
                            turret.setPower(0.0);
                        if (right && (double) turret.getCurrentPosition() <= turretPos)
                            turret.setPower(0.0);
                         */
                    }
                }

                correction = checkDirection();
                leftFront.setPower(-1 * (power + correction));
                rightFront.setPower(-1 * (power - correction));
                leftRear.setPower(-1 * (power + correction));
                rightRear.setPower(-1 * (power - correction));

                if (up && (double) (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2  >= slideHeight) {
                    slideLeft.setPower(0.0);
                    slideRight.setPower(0.0);
                }
                if (down && (double) (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2  <= slideHeight) {
                    slideLeft.setPower(0.0);
                    slideRight.setPower(0.0);
                }

                /* if (left && (double) turret.getCurrentPosition() >= turretPos)
                    turret.setPower(0.0);
                if (right && (double) turret.getCurrentPosition() <= turretPos)
                    turret.setPower(0.0);
                 */

                Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                opMode.telemetry.addData("Gyro Angle", "(%.2f)", angles.firstAngle);
                opMode.telemetry.addData("Encoders:", "M0: %3d  M1:%3d  M2:%3d  M3:%3d",
                        leftFront.getCurrentPosition(),
                        rightFront.getCurrentPosition(),
                        leftRear.getCurrentPosition(),
                        rightRear.getCurrentPosition());
                opMode.telemetry.update();

                if ((Math.abs(leftFront.getCurrentPosition()) + Math.abs(rightFront.getCurrentPosition()) +
                        Math.abs(leftRear.getCurrentPosition()) + Math.abs(rightRear.getCurrentPosition())) / 4.0 > ((4.0/6) * motorDistance) && state2)
                {
                    opMode.telemetry.addData("turretPos", turret.getCurrentPosition());
                    opMode.telemetry.update();
                    for (double j = 1; j >= 0; j -= .0625) {
                        correction = checkDirection();
                        leftFront.setPower(-j * (power + correction));
                        rightFront.setPower(-j * (power - correction));
                        leftRear.setPower(-j * (power + correction));
                        rightRear.setPower(-j * (power - correction));
                        state2 = false;
                        if (up && (double) (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2 >= slideHeight) {
                            slideLeft.setPower(0.0);
                            slideRight.setPower(0.0);
                        }
                        if (down && (double) (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2 <= slideHeight) {
                            slideLeft.setPower(0.0);
                            slideRight.setPower(0.0);
                        }
                        /* if (left && (double) turret.getCurrentPosition() >= turretPos)
                            turret.setPower(0.0);
                        if (right && (double) turret.getCurrentPosition() <= turretPos)
                            turret.setPower(0.0);

                         */
                    }
                }

                // Stop driving when Motor Encoder Avg. <= motorDistance
                if ((Math.abs(leftFront.getCurrentPosition()) + Math.abs(rightFront.getCurrentPosition()) +
                        Math.abs(leftRear.getCurrentPosition()) + Math.abs(rightRear.getCurrentPosition()) / 4.0
                        >= Math.abs(motorDistance))) {
                    leftFront.setPower(0.0);
                    rightFront.setPower(0.0);
                    leftRear.setPower(0.0);
                    rightRear.setPower(0.0);
                    break;
                }
            }
        }
        if (slidePower != 0) {
            while (true) {
                if (slideLeft.getCurrentPosition() < slideHeight) {
                    slideLeft.setPower(-slidePower);
                    slideRight.setPower(-slidePower);
                    up = true;
                } else {
                    slideLeft.setPower(slidePower);
                    slideRight.setPower(slidePower);
                    down = true;
                }
                if (up && (double) (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2 >= slideHeight) {
                    slideLeft.setPower(0.0);
                    slideRight.setPower(0.0);
                    break;
                }
                if (down && (double) (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2 <= slideHeight) {
                    slideLeft.setPower(0.0);
                    slideRight.setPower(0.0);
                    break;
                }
                opMode.telemetry.addData("slidePos", "position: %3d", slideLeft.getCurrentPosition());
                opMode.telemetry.addData("turretPos", "position: %3d", turret.getCurrentPosition());
                opMode.telemetry.update();
            }
        }

        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideLeft.setPower(0);
        slideRight.setPower(0);
        turret.setPower(0);
    }

    public void accelLeftGyroSlidesTurret(double inchesToDrive, double drivePower, double slideHeight, double slidePower, int turretPos, double turretSpeed) {

        double power = drivePower;
        double motorDistance = (double) (inchesToDrive * STRAFE_COUNTS_PER_INCH);
        double correction;
        boolean state1 = true;
        boolean state2 = true;
        boolean up = false;
        boolean down = false;
        boolean left = false;
        boolean right = false;

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideLeft.setTargetPosition((int) slideHeight);
        slideRight.setTargetPosition((int) slideHeight);
        turret.setTargetPosition(turretPos);

        //slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        opMode.sleep(150);

        // move slides
        if (slideLeft.getCurrentPosition() < slideHeight) {
            slideLeft.setPower(-slidePower);
            slideRight.setPower(-slidePower);
            up = true;
        } else {
            slideLeft.setPower(slidePower);
            slideRight.setPower(slidePower);
            down = true;
        }

        //move turret
        if (turret.getCurrentPosition() > turretPos)
            turret.setPower(-turretSpeed);
        else if (turret.getCurrentPosition() < turretPos)
            turret.setPower(turretSpeed);
        else if (turret.getCurrentPosition() == turretPos)
            turret.setPower(0);

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            startMotorCounts = rightFront.getCurrentPosition();
            stopMotorCounts = startMotorCounts + (int) (inchesToDrive * STRAFE_COUNTS_PER_INCH);
            drive1 = 0.0;
            drive2 = power;
            leftPower = Range.clip(drive1 + drive2, -1.0, 1.0);
            rightPower = Range.clip(drive1 - drive2, -1.0, 1.0);
            // Send calculated power to wheels
            /* leftFront.setPower(rightPower);
            rightFront.setPower(leftPower);
            leftRear.setPower(leftPower);
            rightRear.setPower(rightPower);
            
             */

            while (true) {

                if ((Math.abs(leftFront.getCurrentPosition()) + Math.abs(rightFront.getCurrentPosition()) +
                        Math.abs(leftRear.getCurrentPosition()) + Math.abs(rightRear.getCurrentPosition())) / 4.0 < (Math.abs(stopMotorCounts) / 6)
                        && state1)
                {
                    for (double i = 0; i <= 1; i += .0625) {
                        correction = checkDirection();
                        leftFront.setPower(i * (rightPower - correction));
                        rightFront.setPower(i * (leftPower + correction));
                        leftRear.setPower(i * (leftPower - correction));
                        rightRear.setPower(i * (rightPower + correction));
                        state1 = false;
                        if (up && (double) (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2  >= slideHeight) {
                            slideLeft.setPower(0.0);
                            slideRight.setPower(0.0);
                        }
                        if (down && (double) (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2  <= slideHeight) {
                            slideLeft.setPower(0.0);
                            slideRight.setPower(0.0);
                        }
                        /* if (left && (double) turret.getCurrentPosition() >= turretPos)
                            turret.setPower(0.0);
                        if (right && (double) turret.getCurrentPosition() <= turretPos)
                            turret.setPower(0.0);

                         */
                    }
                }

                correction = checkDirection();
                leftFront.setPower(rightPower - correction);
                rightFront.setPower(leftPower + correction);
                leftRear.setPower(leftPower - correction);
                rightRear.setPower(rightPower + correction);

                if (up && (double) (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2  >= slideHeight) {
                    slideLeft.setPower(0.0);
                    slideRight.setPower(0.0);
                }
                if (down && (double) (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2  <= slideHeight) {
                    slideLeft.setPower(0.0);
                    slideRight.setPower(0.0);
                }
                /* if (left && (double) turret.getCurrentPosition() >= turretPos)
                    turret.setPower(0.0);
                if (right && (double) turret.getCurrentPosition() <= turretPos)
                    turret.setPower(0.0);

                 */

                Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                opMode.telemetry.addData("Gyro Angle", "(%.2f)", angles.firstAngle);
                opMode.telemetry.addData("Encoders:", "M0: %3d  M1:%3d  M2:%3d  M3:%3d",
                        leftFront.getCurrentPosition(),
                        -rightFront.getCurrentPosition(),
                        -leftRear.getCurrentPosition(),
                        rightRear.getCurrentPosition());
                opMode.telemetry.update();

                if ((Math.abs(leftFront.getCurrentPosition()) + Math.abs(rightFront.getCurrentPosition()) +
                        Math.abs(leftRear.getCurrentPosition()) + Math.abs(rightRear.getCurrentPosition())) / 4.0 > ((4.0/6) * Math.abs(stopMotorCounts))
                        && state2)
                {
                    opMode.telemetry.addData("turretPos", turret.getCurrentPosition());
                    opMode.telemetry.update();
                    for (double j = 1; j >= 0; j -= .0625) {
                        correction = checkDirection();
                        leftFront.setPower(j * (rightPower - correction));
                        rightFront.setPower(j * (leftPower + correction));
                        leftRear.setPower(j * (leftPower - correction));
                        rightRear.setPower(j * (rightPower + correction));
                        state2 = false;
                        if (up && (double) (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2  >= slideHeight) {
                            slideLeft.setPower(0.0);
                            slideRight.setPower(0.0);
                        }
                        if (down && (double) (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2  <= slideHeight) {
                            slideLeft.setPower(0.0);
                            slideRight.setPower(0.0);
                        }
                        /* if (left && (double) turret.getCurrentPosition() >= turretPos)
                            turret.setPower(0.0);
                        if (right && (double) turret.getCurrentPosition() <= turretPos)
                            turret.setPower(0.0);

                         */
                    }
                }

                // Stop when the bot has driven the requested distance
                if ((Math.abs(leftFront.getCurrentPosition()) + Math.abs(rightFront.getCurrentPosition()) +
                        Math.abs(leftRear.getCurrentPosition()) + Math.abs(rightRear.getCurrentPosition())) / 4.0 >= Math.abs(stopMotorCounts)) {
                    stop();
                    break;
                }
            }
            if (slidePower != 0) {
                while (true) {
                    if (slideLeft.getCurrentPosition() < slideHeight) {
                        slideLeft.setPower(-slidePower);
                        slideRight.setPower(-slidePower);
                        up = true;
                    } else {
                        slideLeft.setPower(slidePower);
                        slideRight.setPower(slidePower);
                        down = true;
                    }
                    if (up && (double) (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2 >= slideHeight) {
                        slideLeft.setPower(0.0);
                        slideRight.setPower(0.0);
                        break;
                    }
                    if (down && (double) (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2 <= slideHeight) {
                        slideLeft.setPower(0.0);
                        slideRight.setPower(0.0);
                        break;
                    }
                    opMode.telemetry.addData("slidePos", "position: %3d", slideLeft.getCurrentPosition());
                    opMode.telemetry.addData("turretPos", "position: %3d", turret.getCurrentPosition());
                    opMode.telemetry.update();
                }
            }

            turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            slideLeft.setPower(0);
            slideRight.setPower(0);
            turret.setPower(0);
        }
    }

    public void accelRightGyroSlidesTurret(double inchesToDrive, double drivePower, double slideHeight, double slidePower, int turretPos, double turretSpeed) {

        double power = drivePower;
        double motorDistance = (double) (inchesToDrive * STRAFE_COUNTS_PER_INCH);
        double correction;
        boolean state1 = true;
        boolean state2 = true;
        boolean up = false;
        boolean down = false;
        boolean left = false;
        boolean right = false;

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideLeft.setTargetPosition((int) slideHeight);
        slideRight.setTargetPosition((int) slideHeight);

        //slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        turret.setTargetPosition(turretPos);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        opMode.sleep(150);

        // move slides
        if (slideLeft.getCurrentPosition() < slideHeight) {
            slideLeft.setPower(-slidePower);
            slideRight.setPower(-slidePower);
            up = true;
        }
        else if (slideLeft.getCurrentPosition() > slideHeight) {
            slideLeft.setPower(slidePower);
            slideRight.setPower(slidePower);
            down = true;
        }

        //move turret
        if (turret.getCurrentPosition() > turretPos)
            turret.setPower(-turretSpeed);
        else if (turret.getCurrentPosition() < turretPos)
            turret.setPower(turretSpeed);

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            startMotorCounts = rightFront.getCurrentPosition();
            stopMotorCounts = startMotorCounts - (inchesToDrive * STRAFE_COUNTS_PER_INCH);
            drive1 = 0.0;
            drive2 = -power;
            leftPower = Range.clip(drive1 + drive2, -1.0, 1.0);
            rightPower = Range.clip(drive1 - drive2, -1.0, 1.0);
            // Send calculated power to wheels
            /* leftFront.setPower(rightPower);
            rightFront.setPower(leftPower);
            leftRear.setPower(leftPower);
            rightRear.setPower(rightPower);
            
             */

            while (true) {

                if ((Math.abs(leftFront.getCurrentPosition()) + Math.abs(rightFront.getCurrentPosition()) +
                        Math.abs(leftRear.getCurrentPosition()) + Math.abs(rightRear.getCurrentPosition())) / 4.0 < (Math.abs(stopMotorCounts) / 6)
                        && state1)
                {
                    for (double i = 0; i <= 1; i += .0625) {
                        correction = checkDirection();
                        leftFront.setPower(i * (rightPower - correction));
                        rightFront.setPower(i * (leftPower + correction));
                        leftRear.setPower(i * (leftPower - correction));
                        rightRear.setPower(i * (rightPower + correction));
                        state1 = false;
                        if (up && (double) (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2  >= slideHeight) {
                            slideLeft.setPower(0.0);
                            slideRight.setPower(0.0);
                        }
                        if (down && (double) (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2  <= slideHeight) {
                            slideLeft.setPower(0.0);
                            slideRight.setPower(0.0);
                        }
                        /* if (left && (double) turret.getCurrentPosition() >= turretPos)
                            turret.setPower(0.0);
                        if (right && (double) turret.getCurrentPosition() <= turretPos)
                            turret.setPower(0.0);

                         */
                    }
                }

                correction = checkDirection();
                leftFront.setPower(rightPower - correction);
                rightFront.setPower(leftPower + correction);
                leftRear.setPower(leftPower - correction);
                rightRear.setPower(rightPower + correction);

                if (up && (double) (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2  >= slideHeight) {
                    slideLeft.setPower(0.0);
                    slideRight.setPower(0.0);
                }
                if (down && (double) (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2  <= slideHeight) {
                    slideLeft.setPower(0.0);
                    slideRight.setPower(0.0);
                }
                /* if (left && (double) turret.getCurrentPosition() >= turretPos)
                    turret.setPower(0.0);
                if (right && (double) turret.getCurrentPosition() <= turretPos)
                    turret.setPower(0.0);

                 */

                Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                opMode.telemetry.addData("Gyro Angle", "(%.2f)", angles.firstAngle);
                opMode.telemetry.addData("Encoders:", "M0: %3d  M1:%3d  M2:%3d  M3:%3d",
                        leftFront.getCurrentPosition(),
                        -rightFront.getCurrentPosition(),
                        -leftRear.getCurrentPosition(),
                        rightRear.getCurrentPosition());
                opMode.telemetry.update();

                if ((Math.abs(leftFront.getCurrentPosition()) + Math.abs(rightFront.getCurrentPosition()) +
                        Math.abs(leftRear.getCurrentPosition()) + Math.abs(rightRear.getCurrentPosition())) / 4.0 > ((4.0/6) * Math.abs(stopMotorCounts))
                        && state2)
                {
                    opMode.telemetry.addData("turretPos", turret.getCurrentPosition());
                    opMode.telemetry.update();
                    for (double j = 1; j >= 0; j -= .0625) {
                        correction = checkDirection();
                        leftFront.setPower(j * (rightPower - correction));
                        rightFront.setPower(j * (leftPower + correction));
                        leftRear.setPower(j * (leftPower - correction));
                        rightRear.setPower(j * (rightPower + correction));
                        state2 = false;
                        if (up && (double) (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2  >= slideHeight) {
                            slideLeft.setPower(0.0);
                            slideRight.setPower(0.0);
                        }
                        if (down && (double) (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2  <= slideHeight) {
                            slideLeft.setPower(0.0);
                            slideRight.setPower(0.0);
                        }
                        /* if (left && (double) turret.getCurrentPosition() >= turretPos)
                            turret.setPower(0.0);
                        if (right && (double) turret.getCurrentPosition() <= turretPos)
                            turret.setPower(0.0);

                         */
                    }
                }

                // Stop when the bot has driven the requested distance
                if (Math.abs(rightFront.getCurrentPosition()) >= Math.abs(stopMotorCounts)) {
                    stop();
                    break;
                }
            }
            if (slidePower != 0) {
                while (true) {
                    if (slideLeft.getCurrentPosition() < slideHeight) {
                        slideLeft.setPower(-slidePower);
                        slideRight.setPower(-slidePower);
                        up = true;
                    } else {
                        slideLeft.setPower(slidePower);
                        slideRight.setPower(slidePower);
                        down = true;
                    }
                    if (up && (double) (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2 >= slideHeight) {
                        slideLeft.setPower(0.0);
                        slideRight.setPower(0.0);
                        break;
                    }
                    if (down && (double) (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2 <= slideHeight) {
                        slideLeft.setPower(0.0);
                        slideRight.setPower(0.0);
                        break;
                    }
                    opMode.telemetry.addData("slidePos", "position: %3d", slideLeft.getCurrentPosition());
                    opMode.telemetry.addData("turretPos", "position: %3d", turret.getCurrentPosition());
                    opMode.telemetry.update();
                }
            }

            turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            slideLeft.setPower(0);
            slideRight.setPower(0);
            turret.setPower(0);
        }
    }

    public void coneStackDetect(boolean isRed, boolean isLeft) { //120-180 gray, 12-60 red, 210-225 blue
        float[] hsvValues = new float[3];
        int leftEdge = 0;
        int rightEdge = 0;

        if ((isRed && !isLeft) || (!isRed && isLeft)) { //red right and blue left
            driveStraightGyro(3,0.25);
            Color.colorToHSV(colors.toColor(), hsvValues);
            if (hsvValues[0] < 110 || hsvValues[0] > 190) {
                rightEdge = leftFront.getCurrentPosition();
            } if (hsvValues[0] > 110 && hsvValues[0] < 190) {
                leftEdge = leftFront.getCurrentPosition();
            }
            double avgPos = (double)(leftEdge + rightEdge)/2;
            strafeRightIMU(0.5, 12);
        } else { //red left and blue right
            driveStraightGyro(3,0.25);
            Color.colorToHSV(colors.toColor(), hsvValues);
            if (hsvValues[0] < 110 || hsvValues[0] > 190) {
                rightEdge = leftFront.getCurrentPosition();
            } if (hsvValues[0] > 110 && hsvValues[0] < 190) {
                leftEdge = leftFront.getCurrentPosition();
            }
            double avgPos = (double)(leftEdge + rightEdge)/2;
            strafeLeftIMU(0.5, 12);
        }


    }

    public void coneLineDetect(boolean isRed, boolean frontOrBack) {
        float[] hsvValues = {0F, 0F, 0F};
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
        boolean seenCol = false;
        runtime.reset();
        if (isRed) {
            while(true) {
                if (!frontOrBack) {
                    Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
                    leftFront.setPower(-0.1);
                    rightFront.setPower(-0.1);
                    leftRear.setPower(-0.1);
                    rightRear.setPower(-0.1);
                    opMode.telemetry.addData("Hue", hsvValues[0]);
                    opMode.telemetry.update();

                    if (hsvValues[0] < 80 || hsvValues[0] > 190) { //If the robot is starting on gray, it'll go until it sees red, then go until it sees gray again
                        seenCol = true;
                        while (true) {
                            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
                            leftFront.setPower(-0.1);
                            rightFront.setPower(-0.1);
                            leftRear.setPower(-0.1);
                            rightRear.setPower(-0.1);
                            opMode.sleep(200);
                            opMode.telemetry.addData("Hue", hsvValues[0]);
                            opMode.telemetry.update();

                            if (hsvValues[0] < 2 || hsvValues[0] > 65){
                                stop();
                                break;
                            }
                        }
                    }
                    if ((seenCol) && (hsvValues[0] < 2 || hsvValues[0] > 65)) // If the robot is starting on red, it'll stop once it reaches the gray again
                        break;

                } else {
                    Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
                    leftFront.setPower(0.1);
                    rightFront.setPower(0.1);
                    leftRear.setPower(0.1);
                    rightRear.setPower(0.1);
                    opMode.telemetry.addData("Hue", hsvValues[0]);
                    opMode.telemetry.update();

                    if (hsvValues[0] < 80 || hsvValues[0] > 190) { //If the robot is starting on gray, it'll go until it sees red, then go until it sees gray again
                        stop();
                        break;
                        /* seenCol = true;
                        while (true) {
                            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
                            leftFront.setPower(-0.1);
                            rightFront.setPower(-0.1);
                            leftRear.setPower(-0.1);
                            rightRear.setPower(-0.1);
                            opMode.telemetry.addData("Hue", hsvValues[0]);
                            opMode.telemetry.update();

                            if (hsvValues[0] < 2 || hsvValues[0] > 65){
                                stop();
                                break;
                            }
                        }

                         */
                    }

                    if (runtime.seconds() > 3) {
                        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
                        leftFront.setPower(-0.1);
                        rightFront.setPower(-0.1);
                        leftRear.setPower(-0.1);
                        rightRear.setPower(-0.1);
                        opMode.telemetry.addData("Hue", hsvValues[0]);
                        opMode.telemetry.update();

                        if (hsvValues[0] > 2 || hsvValues[0] < 65){
                            stop();
                            break;
                        }
                    }
                }
            }
        }

        else {
            while(true) {
                if (!frontOrBack) {
                    Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
                    leftFront.setPower(-0.1);
                    rightFront.setPower(-0.1);
                    leftRear.setPower(-0.1);
                    rightRear.setPower(-0.1);
                    opMode.telemetry.addData("Hue", hsvValues[0]);
                    opMode.telemetry.update();

                    if (hsvValues[0] < 80 || hsvValues[0] > 190) {
                        seenCol = true;
                        while (true) {
                            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
                            leftFront.setPower(-0.1);
                            rightFront.setPower(-0.1);
                            leftRear.setPower(-0.1);
                            rightRear.setPower(-0.1);
                            opMode.telemetry.addData("Hue", hsvValues[0]);
                            opMode.telemetry.update();

                            if (hsvValues[0] < 200 || hsvValues[0] > 235){
                                stop();
                                break;
                            }
                        }
                    }
                    if ((seenCol) && (hsvValues[0] < 200 || hsvValues[0] > 235))
                        break;

                    if (runtime.seconds() > 3) {
                        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
                        leftFront.setPower(0.1);
                        rightFront.setPower(0.1);
                        leftRear.setPower(0.1);
                        rightRear.setPower(0.1);
                        opMode.telemetry.addData("Hue", hsvValues[0]);
                        opMode.telemetry.update();

                        if (hsvValues[0] > 2 || hsvValues[0] < 65){
                            stop();
                            break;
                        }
                    }
                } else {
                    Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
                    leftFront.setPower(0.1);
                    rightFront.setPower(0.1);
                    leftRear.setPower(0.1);
                    rightRear.setPower(0.1);
                    opMode.telemetry.addData("Hue", hsvValues[0]);
                    opMode.telemetry.update();

                    if (hsvValues[0] < 80 || hsvValues[0] > 190) {
                        stop();
                        break;
                    }
                        /* seenCol = true;
                        while (true) {
                            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
                            leftFront.setPower(-0.1);
                            rightFront.setPower(-0.1);
                            leftRear.setPower(-0.1);
                            rightRear.setPower(-0.1);
                            opMode.telemetry.addData("Hue", hsvValues[0]);
                            opMode.telemetry.update();

                            if (hsvValues[0] < 200 || hsvValues[0] > 235){
                                stop();
                                break;
                            }
                        }
                    }
                    if ((seenCol) && (hsvValues[0] < 200 || hsvValues[0] > 235))
                        break;

                         */

                    if (runtime.seconds() > 3) {
                        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
                        leftFront.setPower(-0.1);
                        rightFront.setPower(-0.1);
                        leftRear.setPower(-0.1);
                        rightRear.setPower(-0.1);
                        opMode.telemetry.addData("Hue", hsvValues[0]);
                        opMode.telemetry.update();

                        if (hsvValues[0] > 2 || hsvValues[0] < 65){
                            stop();
                            break;
                        }
                    }
                }
            }
        }


        /* while (!opMode.isStopRequested()) {

            // send the info back to driver station using telemetry function.
            opMode.telemetry.addData("Clear", colorSensor.alpha());
            opMode.telemetry.addData("Red  ", colorSensor.red());
            opMode.telemetry.addData("Green", colorSensor.green());
            opMode.telemetry.addData("Blue ", colorSensor.blue());
            opMode.telemetry.addData("Hue", hsvValues[0]);

            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });



            opMode.telemetry.update();
        }
        */

    }

    public void strafeLeftGyroSlidesTurret(double inchesToDrive, double drivePower, double slideHeight, double slidePower, int turretPos, double turretSpeed) {

        double power = drivePower;
        double motorDistance = (double) (inchesToDrive * WHEEL_COUNTS_PER_INCH);
        double correction;
        boolean up = false;
        boolean down = false;

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //slideLeft.setTargetPosition((int) slideHeight);
        //slideRight.setTargetPosition((int) slideHeight);
        turret.setTargetPosition(turretPos);

        //slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        opMode.sleep(250);

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            opMode.sleep(500);

            if (slideLeft.getCurrentPosition() < slideHeight) {
                slideLeft.setPower(-slidePower);
                slideRight.setPower(-slidePower);
                up = true;
            }
            else if (slideLeft.getCurrentPosition() > slideHeight) {
                slideLeft.setPower(slidePower);
                slideRight.setPower(slidePower);
                down = true;
            }

            startMotorCounts = rightFront.getCurrentPosition();
            stopMotorCounts = startMotorCounts - (int) (inchesToDrive * STRAFE_COUNTS_PER_INCH);
            drive1 = 0.0;
            drive2 = power;
            double lPower = Range.clip(drive1 + drive2, -1.0, 1.0);
            double rPower = Range.clip(drive1 - drive2, -1.0, 1.0);
            // Send calculated power to wheels
            leftFront.setPower(rPower);
            rightFront.setPower(lPower);
            leftRear.setPower(lPower);
            rightRear.setPower(rPower);


            //move turret
            if (turret.getCurrentPosition() > turretPos)
                turret.setPower(-0.2);
            else if (turret.getCurrentPosition() < turretPos)
                turret.setPower(0.2);

            while (true) {

                correction = checkDirection();
                leftFront.setPower(rPower - correction);
                rightFront.setPower(lPower + correction);
                leftRear.setPower(lPower - correction);
                rightRear.setPower(rPower + correction);

                Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                opMode.telemetry.addData("Gyro Angle", "(%.2f)", angles.firstAngle);
                opMode.telemetry.addData("Encoders:", "M0: %3d  M1:%3d  M2:%3d  M3:%3d",
                        leftFront.getCurrentPosition(),
                        -rightFront.getCurrentPosition(),
                        -leftRear.getCurrentPosition(),
                        rightRear.getCurrentPosition());
                opMode.telemetry.update();

                if (up && (double) (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2  >= slideHeight) {
                    slideLeft.setPower(0.0);
                    slideRight.setPower(0.0);
                }
                if (down && (double) (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2  <= slideHeight) {
                    slideLeft.setPower(0.0);
                    slideRight.setPower(0.0);
                }

                // Stop when the bot has driven the requested distance
                if (Math.abs(rightFront.getCurrentPosition()) >= Math.abs(stopMotorCounts)) {
                    stop();
                    break;
                }
            }
        }
    }

    public void strafeRightGyroSlidesTurret(double inchesToDrive, double drivePower, double slideHeight, double slidePower, int turretPos, double turretSpeed) {

        double power = drivePower;
        double motorDistance = (double) (inchesToDrive * WHEEL_COUNTS_PER_INCH);
        double correction;

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideLeft.setTargetPosition((int) slideHeight);
        slideRight.setTargetPosition((int) -slideHeight);
        turret.setTargetPosition(turretPos);

        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        opMode.sleep(250);

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            opMode.sleep(500);

            startMotorCounts = rightFront.getCurrentPosition();
            stopMotorCounts = startMotorCounts + (int) (inchesToDrive * STRAFE_COUNTS_PER_INCH);
            drive1 = 0.0;
            drive2 = -power;
            leftPower = Range.clip(drive1 + drive2, -1.0, 1.0);
            rightPower = Range.clip(drive1 - drive2, -1.0, 1.0);
            // Send calculated power to wheels
            leftFront.setPower(rightPower);
            rightFront.setPower(leftPower);
            leftRear.setPower(leftPower);
            rightRear.setPower(rightPower);

            // move slides
            if (slideLeft.getCurrentPosition() < slideHeight) {
                slideLeft.setPower(slidePower);
                slideRight.setPower(-slidePower);
            }
            else if (slideLeft.getCurrentPosition() > slideHeight) {
                slideLeft.setPower(-slidePower);
                slideRight.setPower(slidePower);
            }

            //move turret
            if (turret.getCurrentPosition() > turretPos)
                turret.setPower(-0.2);
            else if (turret.getCurrentPosition() < turretPos)
                turret.setPower(0.2);

            while (true) {

                correction = checkDirection();
                leftFront.setPower(rightPower - correction);
                rightFront.setPower(leftPower + correction);
                leftRear.setPower(leftPower - correction);
                rightRear.setPower(rightPower + correction);

                Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                opMode.telemetry.addData("Gyro Angle", "(%.2f)", angles.firstAngle);
                opMode.telemetry.addData("Encoders:", "M0: %3d  M1:%3d  M2:%3d  M3:%3d",
                        leftFront.getCurrentPosition(),
                        -rightFront.getCurrentPosition(),
                        -leftRear.getCurrentPosition(),
                        rightRear.getCurrentPosition());
                opMode.telemetry.update();

                // Stop when the bot has driven the requested distance
                if (Math.abs(rightFront.getCurrentPosition()) >= Math.abs(stopMotorCounts)) {
                    stop();
                    break;
                }
            }
        }
    }

    @Override
    public double getRawExternalHeading() { return 0; }

    @Override
    public Double getExternalHeadingVelocity() {
        return (double) imu.getAngularVelocity().zRotationRate;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public void stop() {
        if (leftFront != null && rightFront != null && leftRear != null && rightRear != null) {
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);
            this.leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            this.rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            this.leftRear.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            this.rightRear.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }
}
