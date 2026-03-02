package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.robotcore.hardware.Servo;
public class Led {
    private final Servo LedLed;

    public Led(Servo LedLed) {
        this.LedLed = LedLed;
    }

    public void setColor(double color) {
        LedLed.setPosition(color);
    }
}
