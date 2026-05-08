package org.firstinspires.ftc.teamcode.Teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.DriveToPoint;
import org.firstinspires.ftc.teamcode.Subsystems.Climber;
import org.firstinspires.ftc.teamcode.Subsystems.DrivetrainFO;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeThing;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Led;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Locale;


@TeleOp(name = "OdoOp")

public class OdoOp extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
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
    double xStartingPosition = 0.0;
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
        frontLeft.setZeroPowerBehavior(BRAKE);
        frontRight.setZeroPowerBehavior(BRAKE);
        backLeft.setZeroPowerBehavior(BRAKE);
        backRight.setZeroPowerBehavior(BRAKE);

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
        odo.setOffsets(3.5, 17, DistanceUnit.CM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        initAprilTag();
        // wait for calibration
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

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

        telemetry.addData("Status", "Running");
        telemetry.update();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                odo.update();
                telemetryAprilTag();


                double y = -this.gamepad1.left_stick_y;
                double x = this.gamepad1.left_stick_x;
                double r = this.gamepad1.right_stick_x;

                double botHeading = odo.getPosition().getHeading(AngleUnit.RADIANS);
                double rotx = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double roty = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
                rotx = rotx * 1.1;
                Pose2D currentPos = odo.getPosition();
                String posData = String.format(Locale.US, "Pos X: %.1f, Pos Y: %.1f, Heading (deg): %.1f°",
                        currentPos.getX(DistanceUnit.INCH),
                        currentPos.getY(DistanceUnit.INCH),
                        currentPos.getHeading(AngleUnit.DEGREES));
                telemetry.addData("Odo Position", posData);
                telemetry.addData("Odo Status", odo.getDeviceStatus());

                telemetry.addData("Encoder X raw", odo.getEncoderX());
                telemetry.addData("Encoder Y raw", odo.getEncoderY());

                if (gamepad1.options) {
                    imu.resetYaw();
                    Pose2D cur = odo.getPosition();
                    odo.setPosition(new Pose2D(DistanceUnit.MM,
                            cur.getX(DistanceUnit.MM),
                            cur.getY(DistanceUnit.MM),
                            AngleUnit.DEGREES, 0));
                }

                if (gamepad1.left_bumper) {
                    drivetrain.setState(true);
                } else {
                    drivetrain.setState(false);
                }

                if (gamepad2.y) {
                    intake.setSpeed(1);
                } else if (gamepad2.a) {
                    intake.setSpeed(-1);
                } else {
                    intake.setSpeed(0);
                }
                double targetVelocity = 0;
                if (gamepad2.right_trigger > 0.1) {
                    targetVelocity = 1250;
                } else if (gamepad2.left_trigger > 0.1) {
                    targetVelocity = 1100;
                } else if (gamepad2.right_bumper) {
                    targetVelocity = 1000;
                } else if (gamepad2.left_bumper) {
                    targetVelocity = 800;
                } else {
                    targetVelocity = 0;
                }

                shooter.setVelocity(targetVelocity);

                double m1 = shooterMotor1.getVelocity();
                double m2 = shooterMotor2.getVelocity();
                telemetry.addData("Shooter 1 speed: ", m1);
                telemetry.addData("Shooter 2 speed: ", m2);

                if (targetVelocity > 0 && m1 >= targetVelocity && m2 >= targetVelocity) {
                    led.setColor(0.5);
                    telemetry.addLine("Shooters ready");
                } else {
                    led.setColor(1);
                }

                if (gamepad2.dpad_up) {
                    intakeThing.setSpeed(-0.75);
                } else if (gamepad2.dpad_down) {
                    intakeThing.setSpeed(0.85);
                } else if (gamepad2.dpad_right) {
                    intakeThing.setSpeed(-0.40);
                } else if (gamepad2.dpad_left) {
                    intakeThing.setSpeed(0.40);
                } else {
                    intakeThing.setSpeed(0);
                }
                //if (gamepad2.y) {
                //  climber.setSpeed(1);
                //} else if (gamepad2.b) {
                //    climber.setSpeed(-1);
                //} else {
                //    climber.setSpeed(0);
                //}

                if (gamepad1.b) {
                    alignToTag();
                } else if (gamepad1.a) {
                    driveToTarget(targetPose, 0.5);
                } else {
                    drivetrain.mecanumDrive(rotx, roty, r);
                }

                telemetry.update();
            }
        }
        visionPortal.close();
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
        this.targetPose = targetPose;
        nav.driveTo(odo.getPosition(), targetPose, speed, 0);
        frontLeft.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
        frontRight.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
        backLeft.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
        backRight.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));

        String data2 = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", targetPose.getX(DistanceUnit.INCH), targetPose.getY(DistanceUnit.INCH), targetPose.getHeading(AngleUnit.RADIANS));
        telemetry.addData("TARGET Position", data2);
    }

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                telemetry.addData("Bearing", detection.ftcPose.bearing);
                if (detection.ftcPose.bearing < 3 && detection.ftcPose.bearing > -3) {
                    telemetry.addLine("Tag is centered");
                } else if (detection.ftcPose.bearing >= 3) {
                    telemetry.addLine("Tag is to the left");
                } else {
                    telemetry.addLine("Tag is to the right");
                }
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }

    private void alignToTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections.isEmpty()) {
            drivetrain.mecanumDrive(0, 0, 0);
            return;
        }

        AprilTagDetection tag = detections.get(0);

        double bearingError = tag.ftcPose.bearing; // degrees, positive = tag is to the left
        double yawError = tag.ftcPose.yaw;     // degrees, positive = robot is turned right

        double strafe = bearingError * 0.02;  // tune this gain
        double rotate = -yawError * 0.02;  // tune this gain

        strafe = Range.clip(strafe, -0.4, 0.4);
        rotate = Range.clip(rotate, -0.4, 0.4);

        drivetrain.mecanumDrive(strafe, 0, rotate);

        telemetry.addData("Bearing error", bearingError);
        telemetry.addData("Yaw error", yawError);
    }

}

