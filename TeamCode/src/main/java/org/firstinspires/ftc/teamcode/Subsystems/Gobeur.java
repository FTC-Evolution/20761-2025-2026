package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
public class Gobeur {
    double power;
    private DcMotor gobeurMotor;
    public Gobeur(DcMotor gobeurMotor) {
        this.gobeurMotor = gobeurMotor;
    }
    public void setSpeed(double power) {
        gobeurMotor.setPower(power);
    }
}
