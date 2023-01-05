package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

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

    public static final double AMAX_POS = 0.60;     // Maximum rotational position ---- Phil Claw: 1.4; GoBilda Claw: 1.4
    public static final double AMIN_POS = 0.25;     // Minimum rotational position ---- Phil Claw: 0.7; GoBilda Claw: 0.61
    public double  Aposition = AMIN_POS;                 // Start position

    public static final double BMAX_POS     =  1.00;     // Maximum rotational position
    public static final double BMIN_POS     =  0.40;     // Minimum rotational position
    public double  Bposition = BMIN_POS;                 // Start position

    public static final double CLAW_OPEN_SETTING = AMAX_POS;
    public static final double CLAW_CLOSED_SETTING = AMIN_POS;

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
    public int startMotorCounts = 0;
    public int stopMotorCounts = 0;

    static final double     STRAFE_COUNTS_PER_INCH  = 47.4369230781;
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

    private void resetAngle() {
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

    public void moveTurretToPositionABS(int motorTurretEncoderCounts) {
        int turretHalfRotationCounts = 4000;  // This number is just a guess, need to check on B Bot

        turret.setTargetPosition(motorTurretEncoderCounts);
        turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        if (Math.abs(motorTurretEncoderCounts) < turretHalfRotationCounts) {
            if (turret.getCurrentPosition() < motorTurretEncoderCounts) {
                turret.setPower(0.1);
            } else {
                turret.setPower(-0.1);
            }
        }
    }

    public void moveSlidesToHeightABS(int motorSlideEncoderCounts) {
        double currentTime = runtime.time();

        // opMode.sleep(500);

        slideLeft.setTargetPosition(motorSlideEncoderCounts);
        slideRight.setTargetPosition(-motorSlideEncoderCounts);

        slideLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slideRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        slideLeft.setPower(-0.5);
        slideRight.setPower(0.5);
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
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

        if(((slideLeft.getCurrentPosition()) > 5000 && slidePower < 0) ||
                ((slideLeft.getCurrentPosition()) < 100 && slidePower > 0)){
            slideLeft.setPower(0);
            slideRight.setPower(0);
        }
        else{
            slideLeft.setPower(-0.85 * slidePower);
            slideRight.setPower(0.85 * slidePower);
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

    public void moveSlidesToHeight(int motorSlideEncoderCounts) {
        double currentTime = runtime.time();

        opMode.sleep(500);

        slideLeft.setTargetPosition(motorSlideEncoderCounts);
        slideRight.setTargetPosition(-motorSlideEncoderCounts);

        slideLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slideRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        slideLeft.setPower(-0.85);
        slideRight.setPower(0.85);

        if (slideRight.getCurrentPosition() < motorSlideEncoderCounts) {
            while ((slideRight.getCurrentPosition() <= motorSlideEncoderCounts)) {
                opMode.telemetry.addData("Slide Pos: ", slideRight.getCurrentPosition());
                opMode.telemetry.update();
            }
        }
        else {
            while (slideRight.getCurrentPosition() >= motorSlideEncoderCounts) {
                opMode.telemetry.addData("Slide Pos: ", slideRight.getCurrentPosition());
                opMode.telemetry.update();
            }
        }

        slideLeft.setPower(0.0);
        slideRight.setPower(0.0);

        slideLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slideRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void moveTurret() throws InterruptedException {
        double turretPower = Range.clip(opMode.gamepad2.left_stick_x, -1.0, 1.0);
        double turretPos = turret.getCurrentPosition();
        opMode.telemetry.addData("turretPos", turretPos);

        if ((turretPos >= 850 && turretPower < -0.01) || (turretPos <= -850 && turretPower > 0.01)) {
            turret.setPower(0);
        }
        else {
            turret.setPower(-0.35 * turretPower);
        }
    }

    public void openClaw() {
        Claw.setPosition(CLAW_OPEN_SETTING);
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

        opMode.sleep(250);

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

    public void driveStraightGyroSlidesTurret(double inchesToDrive, double drivePower, double slideHeight, double slidePower, int turretPos, double turretSpeed) {

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

        slideLeft.setTargetPosition((int) slideHeight);
        slideRight.setTargetPosition((int) -slideHeight);
        turret.setTargetPosition(turretPos);

        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        resetAngle();

        opMode.sleep(250);

        if (motorDistance >= 0) {
            // Start driving forward
            leftFront.setPower(0.25);
            rightFront.setPower(0.25);
            leftRear.setPower(0.25);
            rightRear.setPower(0.25);

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

                /* if ((slidePower > 0 && slideLeft.getCurrentPosition() >= slideHeight) || (slidePower < 0 && slideLeft.getCurrentPosition() <= slideHeight)){
                    slideLeft.setPower(0.0);
                    slideRight.setPower(0.0);
                }

                 */

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
    }

    public void strafeLeftGyroSlidesTurret(double inchesToDrive, double drivePower, double slideHeight, double slidePower, int turretPos, double turretSpeed) {

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
            stopMotorCounts = startMotorCounts - (int) (inchesToDrive * STRAFE_COUNTS_PER_INCH);
            drive1 = 0.0;
            drive2 = power;
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
