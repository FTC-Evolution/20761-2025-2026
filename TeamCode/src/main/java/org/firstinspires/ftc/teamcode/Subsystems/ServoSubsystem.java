package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.robotcore.hardware.CRServo;

public class ServoSubsystem {

    private final CRServo servo;
    public ServoSubsystem(CRServo servo) {
        this.servo = servo;
    }
    public void setSpeed(double power) {
        servo.setPower(power);
    }
}
