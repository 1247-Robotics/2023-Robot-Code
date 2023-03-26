package frc.robot;

import edu.wpi.first.wpilibj.Servo;

/*
Just asked ChatGPT for a crash course in OOP in java so not my fault if this breaks something
 */

public class Claw_Servo {
    int PWMPort;
    private Servo servo;
    private int closedAngle;
    private int openAngle;

    public Claw_Servo(int PWMPort) {
        this.PWMPort = PWMPort;
        this.servo = new Servo(PWMPort);

    }

    public void set(int Angle) {
        this.servo.setAngle(Angle);
    }

    public void setClosed(int closed) {
        this.closedAngle = closed;
    }

    public void setOpen(int open) {
        this.openAngle = open;
    }

    public void close() {
        this.servo.setAngle(this.closedAngle);
    }
    
    public void open() {
        this.servo.setAngle(this.openAngle);
    }

    public int getClosed() {
        return this.closedAngle;
    }

    public int getOpen() {
        return this.openAngle;
    }
}
