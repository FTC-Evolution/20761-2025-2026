package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Shooter {

    private final DcMotorEx shooterMotor1;
    private final DcMotorEx shooterMotor2;
    public Shooter(DcMotorEx shooterMotor1, DcMotorEx shooterMotor2) {
        this.shooterMotor1 = shooterMotor1;
        this.shooterMotor2 = shooterMotor2;
    }
    public void setSpeed(double power) {
        shooterMotor1.setPower(power);
        shooterMotor2.setPower(power);
    }
    public void setVelocity(double velocity) {
        shooterMotor1.setVelocity(velocity);
        shooterMotor2.setVelocity(velocity);
    }


}
