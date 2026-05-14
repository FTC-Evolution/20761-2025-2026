package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.DriveToPoint;
import org.firstinspires.ftc.teamcode.Subsystems.DrivetrainFO;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeThing;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Led;

import java.util.Locale;


@Autonomous(name = "Encoder Auto")


public class EncoderAuto extends LinearOpMode {

    DrivetrainFO drivetrain;
    IMU imu;
    Intake intake;
    Shooter shooter;
    //Climber climber;
    IntakeThing intakeThing;
    GoBildaPinpointDriver odo;
    DriveToPoint nav;
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    Led led;
    Pose2D targetPose = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0);
    double xStartingPosition = -0.0;
    double yStartingPosition = 0.0;
    double headingStartingPosition = 0.0;

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)
        );
        imu.initialize(parameters);

        frontLeft = hardwareMap.dcMotor.get("fl");
        frontRight = hardwareMap.dcMotor.get("fr");
        backLeft = hardwareMap.dcMotor.get("bl");
        backRight = hardwareMap.dcMotor.get("br");

        DcMotor intakeMotor = hardwareMap.dcMotor.get("intake");
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        intake = new Intake(intakeMotor);

        DcMotorEx shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooterMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DcMotorEx shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooterMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor1.setDirection(DcMotor.Direction.REVERSE);

        Servo LedServo = hardwareMap.servo.get("led");
        led = new Led(LedServo);

        shooter = new Shooter(shooterMotor1, shooterMotor2);

        //DcMotor climberMotor = hardwareMap.dcMotor.get("climber");
        //climber = new Climber(climberMotor);

        DcMotor intakeThingMotor = hardwareMap.dcMotor.get("intakething");
        intakeThing = new IntakeThing(intakeThingMotor);

        odo = this.hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(0, 0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();

        nav = new DriveToPoint(this);
        nav.setDriveType(DriveToPoint.DriveType.MECANUM);


        drivetrain = new DrivetrainFO(
                frontLeft,
                frontRight,
                backLeft,
                backRight,
                imu
        );

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        odo.setPosition(new Pose2D(DistanceUnit.MM, xStartingPosition, yStartingPosition, AngleUnit.DEGREES, headingStartingPosition));

        waitForStart();

        if (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();

            // put code

            drivetrain.mecanumDrive(0, 0, 0);
        }

    }

    public boolean isAtTarget(Pose2D target, double posTolerance, double velTolerance, double angleTolerance) {
        odo.update();
        return Math.abs(odo.getPosX(DistanceUnit.MM) - target.getX(DistanceUnit.MM)) < posTolerance &&
                Math.abs(odo.getPosY(DistanceUnit.MM) - target.getY(DistanceUnit.MM)) < posTolerance &&
                Math.abs(odo.getPosition().getHeading(AngleUnit.RADIANS) - target.getHeading(AngleUnit.RADIANS)) < angleTolerance &&
                Math.abs(odo.getVelX(DistanceUnit.MM)) < velTolerance &&
                Math.abs(odo.getVelY(DistanceUnit.MM)) < velTolerance;
    }

    public boolean isAtTargetSimple(Pose2D target) {
        return isAtTarget(target, 15, 10, Math.toRadians(3));
    }

    public void driveToTarget(Pose2D targetPose, double speed) {
        odo.update();
        this.targetPose = targetPose;
        nav.driveTo(odo.getPosition(), targetPose, speed, 0);
        frontLeft.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
        frontRight.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
        backLeft.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
        backRight.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));

        String data2 = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", targetPose.getX(DistanceUnit.INCH), targetPose.getY(DistanceUnit.INCH), targetPose.getHeading(AngleUnit.RADIANS));
        telemetry.addData("TARGET Position", data2);
    }

    static final double TICKS_PER_REV = 537.7;  // goBILDA 312 RPM motor
    static final double WHEEL_DIAMETER_MM = 96.0; // your mecanum wheel diameter
    static final double TICKS_PER_MM = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_MM);
    static final double STRAFE_CORRECTION = 1.1;

    public void driveForwaard(double mm, double speed) {
        int ticks = (int)(mm * TICKS_PER_MM);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(ticks);
        frontRight.setTargetPosition(ticks);
        backLeft.setTargetPosition(ticks);
        backRight.setTargetPosition(ticks);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        while (opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {
            telemetry.addData("Driving forward", "%d ticks", ticks);
            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void strafe(double mm, double speed) {
        int ticks = (int)(mm * TICKS_PER_MM * STRAFE_CORRECTION);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // strafe macenum
        frontLeft.setTargetPosition(ticks);
        frontRight.setTargetPosition(-ticks);
        backLeft.setTargetPosition(-ticks);
        backRight.setTargetPosition(ticks);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        while (opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {
            telemetry.addData("Strafing", "%d ticks", ticks);
            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turnTo(double targetDegrees, double speed) {
        double error = normalizeAngle(targetDegrees - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        while (opModeIsActive() && Math.abs(error) > 2.0) {
            error = normalizeAngle(targetDegrees - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            double turnPower = Math.max(0.15, Math.min(speed, Math.abs(error) / 90.0 * speed));
            double dir = Math.signum(error);

            // tourner de degrees
            frontLeft.setPower(dir * turnPower);
            frontRight.setPower(-dir * turnPower);
            backLeft.setPower(dir * turnPower);
            backRight.setPower(-dir * turnPower);

            telemetry.addData("Turning to", "%.1f°, error: %.1f°", targetDegrees, error);
            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) {
            angle -= 360;
        }
        while (angle < -180) {
            angle += 360;
        }
        return angle;
    }
}

