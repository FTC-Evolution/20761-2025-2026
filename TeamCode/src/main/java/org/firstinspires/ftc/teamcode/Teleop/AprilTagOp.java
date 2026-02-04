package org.firstinspires.ftc.teamcode.Teleop;

import android.util.Size;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.DriveToPoint;
import org.firstinspires.ftc.teamcode.Subsystems.Climber;
import org.firstinspires.ftc.teamcode.Subsystems.DrivetrainFO;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.ServoSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Locale;


@TeleOp(name = "AprilTagOp")

public class AprilTagOp extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);
    AprilTagProcessor aprilTag;
    VisionPortal visionPortal;
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
    double xStartingPosition = -1828.8;
    double yStartingPosition = 0.0;
    double headingStartingPosition = 0.0;
    @Override
    public void runOpMode() {
        initAprilTag();
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

        DcMotor shooterMotor1 = hardwareMap.dcMotor.get("shooter1");
        DcMotor shooterMotor2 = hardwareMap.dcMotor.get("shooter2");
        shooterMotor1.setDirection(DcMotor.Direction.REVERSE);
        shooter = new Shooter(shooterMotor1, shooterMotor2);

        DcMotor climberMotor = hardwareMap.dcMotor.get("climber");
        climber = new Climber(climberMotor);

        CRServo servoMotor = hardwareMap.crservo.get("servo");
        servo = new ServoSubsystem(servoMotor);
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
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();

        if(opModeIsActive()) {
            while (opModeIsActive()) {
                telemetryAprilTag();
                telemetry.update();
                double y = -this.gamepad1.left_stick_y;
                double x = this.gamepad1.left_stick_x;
                double r = this.gamepad1.right_stick_x;
                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                double rotx = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double roty = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
                rotx = rotx * 1.1;

                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }
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
                if (gamepad1.a) {
                    driveToTarget(targetPose, 0.5);
                }

                drivetrain.mecanumDrive(rotx,roty,r);
                telemetry.update();
            }
            visionPortal.close();
        }

    }
//    public boolean isAtTarget(double posTolerance, double velTolerance, double angleTolerance) {
//        odo.update();
//        return Math.abs(odo.getPosX(DistanceUnit.MM) - targetPose.getX(DistanceUnit.MM)) < posTolerance &&
//                Math.abs(odo.getPosY(DistanceUnit.MM) - targetPose.getY(DistanceUnit.MM)) < posTolerance &&
//                Math.abs(odo.getPosition().getHeading(AngleUnit.RADIANS) - targetPose.getHeading(AngleUnit.RADIANS)) < angleTolerance &&
//                Math.abs(odo.getVelX(DistanceUnit.MM)) < velTolerance &&
//                Math.abs(odo.getVelY(DistanceUnit.MM)) < velTolerance;
//    }

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
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                if (!detection.metadata.name.contains("Obelisk")) {
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                            detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            detection.robotPose.getPosition().z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                }
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }

        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
    }
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.setCameraResolution(new Size(640, 480));

        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        builder.addProcessor(aprilTag);

        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }
}
