package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Shooter {

    private final DcMotor shooterMotor1;
    private final DcMotor shooterMotor2;
    public Shooter(DcMotor shooterMotor1, DcMotor shooterMotor2) {
        this.shooterMotor1 = shooterMotor1;
        this.shooterMotor2 = shooterMotor2;
    }
    public void setSpeed(double power) {
        shooterMotor1.setPower(power);
        shooterMotor2.setPower(power);
    }
}
