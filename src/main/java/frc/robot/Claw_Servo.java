package frc.robot;

import edu.wpi.first.wpilibj.Servo;

/*
Just asked ChatGPT for a crash course in OOP in java so not my fault if this breaks something
 */

public class Claw_Servo {
    private Servo servo;
    private int closedAngle;
    private int openAngle;
    private int current;
    private int currentImmune;

    public Claw_Servo(int PWMPort) { this.servo = new Servo(PWMPort); }

    public void set(int Angle) {
        this.current = Angle;
        this.currentImmune = Angle;
        this.servo.setAngle(Angle);
    }

    public void setClosed(int closed) { this.closedAngle = closed; }

    public void setOpen(int open) { this.openAngle = open; }

    public void close() {
        this.current = this.closedAngle;
        this.servo.setAngle(this.closedAngle);
    }
    
    public void open() {
        this.current = this.openAngle;
        this.servo.setAngle(this.openAngle);
    }

    public void openAsManual() {
        this.current = this.openAngle;
        this.currentImmune = this.openAngle;
        this.servo.setAngle(this.openAngle);
    }

    public void closeAsManual() {
        this.current = this.closedAngle;
        this.currentImmune = this.closedAngle;
        this.servo.setAngle(this.closedAngle);
    }

    public int getClosed() { return this.closedAngle; }

    public int getOpen() { return this.openAngle; }

    public int current() { return this.current; }

    public int getLastManual() { return this.currentImmune; }

    public boolean isOpen() {
        if (this.current == this.openAngle) { return true; } else { return false; }
    }

    public boolean isClosed() {
        if (this.current == this.closedAngle) { return true; } else { return false; }
    }

    public void setOpen(boolean open) {
        if (open) { this.open(); } else { this.close(); }
    }

    public void setAngle(int Angle) { this.set(Angle); }
}
