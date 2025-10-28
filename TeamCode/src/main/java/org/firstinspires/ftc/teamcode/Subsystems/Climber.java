package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Climber {

    private final DcMotor climberMotor;
    public Climber(DcMotor climberMotor) {
        this.climberMotor = climberMotor;
    }
    public void setSpeed(double power) {
        climberMotor.setPower(power);
    }
}
