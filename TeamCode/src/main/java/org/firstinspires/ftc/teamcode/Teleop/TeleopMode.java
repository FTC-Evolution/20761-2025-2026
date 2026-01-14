package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.ServoSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DrivetrainFO;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Climber;


@TeleOp(name = "FieldOriented TeleOp")

public class TeleopMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)
        );
        imu.initialize(parameters);

        DcMotor intakeMotor = hardwareMap.dcMotor.get("intake");
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        Intake intake = new Intake(intakeMotor);
        DcMotor shooterMotor1 = hardwareMap.dcMotor.get("shooter1");
        DcMotor shooterMotor2 = hardwareMap.dcMotor.get("shooter2");
        shooterMotor1.setDirection(DcMotor.Direction.REVERSE);
        Shooter shooter = new Shooter(shooterMotor1, shooterMotor2);
        DcMotor climberMotor = hardwareMap.dcMotor.get("climber");
        Climber climber = new Climber(climberMotor);
        CRServo servoMotor = hardwareMap.crservo.get("servo");
        ServoSubsystem servo = new ServoSubsystem(servoMotor);

        DrivetrainFO drivetrain = new DrivetrainFO(
                hardwareMap.dcMotor.get("fl"),
                hardwareMap.dcMotor.get("fr"),
                hardwareMap.dcMotor.get("bl"),
                hardwareMap.dcMotor.get("br"),
                imu
        );



        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();

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
}
