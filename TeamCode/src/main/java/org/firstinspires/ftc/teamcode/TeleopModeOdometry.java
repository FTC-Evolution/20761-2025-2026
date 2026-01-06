package org.firstinspires.ftc.teamcode;  //Folder

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.Climber;
import org.firstinspires.ftc.teamcode.Subsystems.DrivetrainFO;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.ServoSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import java.util.Locale;


@TeleOp(name = "FieldOriented TeleOp")   //Mode

public class TeleopModeOdometry extends LinearOpMode {  // Basic code here

    DrivetrainFO drivetrain;
    IMU imu;
    Intake intake;
    Shooter shooter;
    Climber climber;
    ServoSubsystem servo;
    GoBildaPinpointDriver odo;
    DriveToPoint nav;
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    Pose2D targetPose = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0);

    @Override
    public void runOpMode() {   //run while init
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)
        );
        imu.initialize(parameters);
        frontLeft = hardwareMap.dcMotor.get("fl");
        frontRight = hardwareMap.dcMotor.get("fr");
        backLeft = hardwareMap.dcMotor.get("bl");
        backRight = hardwareMap.dcMotor.get("br");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intake");
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        intake = new Intake(intakeMotor);
        DcMotor shooterMotor1 = hardwareMap.dcMotor.get("shooter1");
        DcMotor shooterMotor2 = hardwareMap.dcMotor.get("shooter2");
        shooterMotor1.setDirection(DcMotor.Direction.REVERSE);
        shooter = new Shooter(shooterMotor1, shooterMotor2);
        DcMotor climberMotor = hardwareMap.dcMotor.get("climber");
        climber = new Climber(climberMotor);
        CRServo servoMotor = hardwareMap.crservo.get("servo");
        servo = new ServoSubsystem(servoMotor);
        DriveToPoint nav;


        nav = new DriveToPoint(this);
        odo = this.hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(0, 0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        nav.setDriveType(DriveToPoint.DriveType.MECANUM);

       drivetrain = new DrivetrainFO(
               frontLeft,
               frontRight,
               backLeft,
               backRight,
               imu
       );



        telemetry.addData("Status", "Initialized");  // print in console
        telemetry.update();

        waitForStart();  // After run when start

        telemetry.addData("Status", "Running");   // print in console
        telemetry.update();

        int pos = 0;

        if(opModeIsActive()) {

            while (opModeIsActive()) {
                double y = -this.gamepad1.left_stick_y;
                double x = this.gamepad1.left_stick_x;
                double r = this.gamepad1.right_stick_x;
                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                double rotx = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double roty = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
                rotx = rotx * 1.1;


             if (gamepad1.options) {
                    imu.resetYaw();
            }

            if (gamepad1.left_bumper) {
                drivetrain.setState(true);
            } else {
                drivetrain.setState(false);
            }

            if (gamepad2.right_trigger > 0.1) {
                intake.setSpeed(1);
            } else {
                intake.setSpeed(0);
            }

            if (gamepad2.right_bumper) {
                shooter.setSpeed(1);
            } else {
                shooter.setSpeed(0);
            }

            if (gamepad2.a) {
                servo.setSpeed(-1);
            } else {
                servo.setSpeed(0);
            }

            if (gamepad2.y) {
                climber.setSpeed(1);
            } else if (gamepad2.b) {
                climber.setSpeed(-1);
            } else {
                climber.setSpeed(0);
            }

            drivetrain.mecanumDrive(rotx,roty,r);
            telemetry.update();
            }

        }

    }
    public boolean isAtTarget(double posTolerance, double velTolerance, double angleTolerance) {
        odo.update();
        return Math.abs(odo.getPosX(DistanceUnit.MM) - targetPose.getX(DistanceUnit.MM)) < posTolerance &&
                Math.abs(odo.getPosY(DistanceUnit.MM) - targetPose.getY(DistanceUnit.MM)) < posTolerance &&
                Math.abs(odo.getPosition().getHeading(AngleUnit.RADIANS) - targetPose.getHeading(AngleUnit.RADIANS)) < angleTolerance &&
                Math.abs(odo.getVelX(DistanceUnit.MM)) < velTolerance &&
                Math.abs(odo.getVelY(DistanceUnit.MM)) < velTolerance;
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
