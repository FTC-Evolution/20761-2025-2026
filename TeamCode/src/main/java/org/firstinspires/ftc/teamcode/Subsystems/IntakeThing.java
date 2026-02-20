package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
public class IntakeThing {

    private final DcMotor intakeThingMotor;
    public IntakeThing(DcMotor intakeThingMotor) {
        this.intakeThingMotor = intakeThingMotor;
    }
    public void setSpeed(double power) {
        intakeThingMotor.setPower(power);
    }
}
