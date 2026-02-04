package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Size;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
import java.util.Locale;

@Autonomous(name = "AprilTagAuto")

public class AprilTagAuto extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
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
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();
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

        telemetry.update();

        initAprilTag();
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();
        // Autonomous code here:
        telemetry.addData("Status", "Running");
        telemetry.update();
        telemetryAprilTag();
        telemetry.update();

        if (gamepad1.dpad_down) {
            visionPortal.stopStreaming();
        } else if (gamepad1.dpad_up) {
            visionPortal.resumeStreaming();
        }
        sleep(20);
        driveToTarget(targetPose, 0.5);
        visionPortal.close();
    }
    private void initAprilTag() {

        aprilTag = new AprilTagProcessor.Builder()
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)

                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        builder.setCameraResolution(new Size(640, 480));

        builder.enableLiveView(true);

        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        builder.addProcessor(aprilTag);

        visionPortal = builder.build();

        //visionPortal.setProcessorEnabled(aprilTag, true);

    }
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                // Only use tags that don't have Obelisk in them
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

}

