package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
public class Intake {

    private final DcMotor intakeMotor;
    public Intake(DcMotor intakeMotor) {
        this.intakeMotor = intakeMotor;
    }
    public void setSpeed(double power) {
        intakeMotor.setPower(power);
    }
}
