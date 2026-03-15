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


@Autonomous(name = "Autonomous Mode")


public class AutonomousMode extends LinearOpMode {

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

            Pose2D shootPos2 = new Pose2D(DistanceUnit.MM, 865.2, 326.6, AngleUnit.DEGREES, 133.2); // replace with correct values

            telemetry.addData("Status", "Driving to shooting position");
            telemetry.update();

            while (!isAtTargetSimple(shootPos2)) {
                driveToTarget(shootPos2, 0.5);
            }

            // shoot balls
            if (isAtTargetSimple(shootPos2)) {
                shooter.setVelocity(1250);

                for (int shot = 1; shot <= 3; shot++) {
                    ElapsedTime spinUpTimer1 = new ElapsedTime();
                    spinUpTimer1.reset();
                    boolean shooterReady = false;

                    while (opModeIsActive() && spinUpTimer1.seconds() < 5.0) {
                        double m1 = shooterMotor1.getVelocity();
                        double m2 = shooterMotor2.getVelocity();
                        telemetry.addData("Shot", "%d/3", shot);
                        telemetry.addData("Shooter Velocity", "Motor 1: %.2f, Motor 2: %.2f", m1, m2);
                        telemetry.addData("Status", "Spinning up shooter");
                        telemetry.update();

                        if (m1 >= 1100 && m2 >= 1100) {
                            shooterReady = true;
                            break;
                        }
                    }

                    if (shooterReady) {
                        telemetry.addData("Status", "Shooter ready, firing shot " + shot);
                    } else {
                        telemetry.addData("Status", "Shooter not up to speed, firing anyway");
                    }
                    telemetry.update();

                    intakeThing.setSpeed(1);
                    sleep(750); // adjust this, this is how much time is needed to feed one ball
                    intakeThing.setSpeed(0);

                    telemetry.addData("Status", "Shot " + shot + " fired");
                    telemetry.update();
                }

                shooter.setSpeed(0);
            }

            // align to balls
            Pose2D ballPos = new Pose2D(DistanceUnit.MM, -900, 300, AngleUnit.DEGREES, 0); // replace with correct values

            telemetry.addData("Status", "Driving to balls");
            telemetry.update();

            while (!isAtTargetSimple(ballPos)) {
                driveToTarget(ballPos, 0.5);
            }

            // intake balls
            intake.setSpeed(1);

            ElapsedTime timer = new ElapsedTime();
            timer.reset();

            while (timer.seconds() < 2) {
                drivetrain.mecanumDrive(0, 0.5, 0);
                telemetry.addData("Status", "Driving forward and intaking");
                telemetry.update();
            }

            drivetrain.mecanumDrive(0,0,0);
            intake.setSpeed(0);

            // align for shooting
            Pose2D shootPos = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0); // replace with correct values

            telemetry.addData("Status", "Driving to shooting position");
            telemetry.update();

            while (!isAtTargetSimple(shootPos)) {
                driveToTarget(shootPos, 0.5);
            }

            // shoot balls
            if (isAtTargetSimple(shootPos)) {
                shooter.setVelocity(1250);

                for (int shot = 1; shot <= 3; shot++) {
                    ElapsedTime spinUpTimer = new ElapsedTime();
                    spinUpTimer.reset();
                    boolean shooterReady = false;

                    while (opModeIsActive() && spinUpTimer.seconds() < 5.0) {
                        double m1 = shooterMotor1.getVelocity();
                        double m2 = shooterMotor2.getVelocity();
                        telemetry.addData("Shot", "%d/3", shot);
                        telemetry.addData("Shooter Velocity", "Motor 1: %.2f, Motor 2: %.2f", m1, m2);
                        telemetry.addData("Status", "Spinning up shooter");
                        telemetry.update();

                        if (m1 > 1200 && m2 > 1200) {
                            shooterReady = true;
                            break;
                        }
                    }

                    if (shooterReady) {
                        telemetry.addData("Status", "Shooter ready, firing shot " + shot);
                    } else {
                        telemetry.addData("Status", "Shooter not up to speed, firing anyway");
                    }
                    telemetry.update();

                    intakeThing.setSpeed(1);
                    sleep(750); // adjust this, this is how much time is needed to feed one ball
                    intakeThing.setSpeed(0);

                    telemetry.addData("Status", "Shot " + shot + " fired");
                    telemetry.update();
                }

                shooter.setSpeed(0);
            }

            // get balls again
            telemetry.addData("Status", "Driving to balls (2)");
            telemetry.update();

            while (!isAtTargetSimple(ballPos)) {
                driveToTarget(ballPos, 0.5);
            }

            // intake balls
            intake.setSpeed(1);

            ElapsedTime timer2 = new ElapsedTime();
            timer2.reset();

            while (timer2.seconds() < 2) {
                drivetrain.mecanumDrive(0, 0.5, 0);
                telemetry.addData("Status", "Driving forward and intaking (2)");
                telemetry.update();
            }

            drivetrain.mecanumDrive(0,0,0);
            intake.setSpeed(0);

            // align for shooting
            telemetry.addData("Status", "Driving to shooting position (2) ");
            telemetry.update();

            while (!isAtTargetSimple(shootPos)) {
                driveToTarget(shootPos, 0.5);
            }

            if (isAtTargetSimple(shootPos)) {
                shooter.setVelocity(1250);

                for (int shot = 1; shot <= 3; shot++) {
                    ElapsedTime spinUpTimer = new ElapsedTime();
                    spinUpTimer.reset();
                    boolean shooterReady = false;

                    while (opModeIsActive() && spinUpTimer.seconds() < 5.0) {
                        double m1 = shooterMotor1.getVelocity();
                        double m2 = shooterMotor2.getVelocity();
                        telemetry.addData("Shot", "%d/3", shot);
                        telemetry.addData("Shooter Velocity", "Motor 1: %.2f, Motor 2: %.2f", m1, m2);
                        telemetry.addData("Status", "Spinning up shooter");
                        telemetry.update();

                        if (m1 > 1200 && m2 > 1200) {
                            shooterReady = true;
                            break;
                        }
                    }

                    if (shooterReady) {
                        telemetry.addData("Status", "Shooter ready, firing shot " + shot);
                    } else {
                        telemetry.addData("Status", "Shooter not up to speed, firing anyway");
                    }
                    telemetry.update();

                    intakeThing.setSpeed(1);
                    sleep(750); // adjust this, this is how much time is needed to feed one ball
                    intakeThing.setSpeed(0);

                    telemetry.addData("Status", "Shot " + shot + " fired");
                    telemetry.update();
                }

                shooter.setSpeed(0);
            }

            // open gate
            Pose2D gatePos = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0); // replace with correct values
            telemetry.addData("Status", "Driving to gate");
            telemetry.update();
            while (!isAtTargetSimple(gatePos)) {
                driveToTarget(gatePos, 0.5);
            }

            if (isAtTargetSimple(gatePos)) {
                // go forward into the lever to open the gate
                drivetrain.mecanumDrive(0, 0.5, 0);
                telemetry.addData("Status", "Opening gate");
                telemetry.update();
                sleep(2000);
            }

            // drive backwards to clear gate

            ElapsedTime timer3 = new ElapsedTime();
            timer3.reset();

            while (timer3.seconds() < 0.25) {
                drivetrain.mecanumDrive(0, -0.2, 0);
                telemetry.addData("Status", "Driving backward");
                telemetry.update();
            }

            drivetrain.mecanumDrive(0,0,0);
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


}

