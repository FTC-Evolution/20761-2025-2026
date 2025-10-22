package org.firstinspires.ftc.teamcode;  //Folder

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.DrivetrainFO;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;

@TeleOp(name = "FieldOriented TeleOp")   //Mode

public class TeleopMode extends LinearOpMode {  // Basic code here

    private DrivetrainFO drivetrain;
    private IMU imu;
    private Intake intake;
    private Shooter shooter;


    @Override
    public void runOpMode() {   //run while init
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)
        );
        imu.initialize(parameters);

        DcMotor intakeMotor = hardwareMap.dcMotor.get("intake");
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        intake = new Intake(intakeMotor);
        DcMotor shooterMotor1 = hardwareMap.dcMotor.get("shooter1");
        DcMotor shooterMotor2 = hardwareMap.dcMotor.get("shooter2");
        shooterMotor2.setDirection(DcMotor.Direction.REVERSE);
        shooter = new Shooter(shooterMotor1, shooterMotor2);

       drivetrain = new DrivetrainFO(
               hardwareMap.dcMotor.get("fl"),
                hardwareMap.dcMotor.get("fr"),
                hardwareMap.dcMotor.get("bl"),
               hardwareMap.dcMotor.get("br"),
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

            if (gamepad2.right_bumper) {
                intake.setSpeed(1);
            } else {
                intake.setSpeed(0);
            }

            if (gamepad2.left_bumper) {
                shooter.setSpeed(1);
            } else {
                shooter.setSpeed(0);
            }

            // if (gamepad2.left_bumper)
            drivetrain.mecanumDrive(rotx,roty,r);
            telemetry.update();
            }
        }
    }
}
