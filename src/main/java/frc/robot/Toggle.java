package frc.robot;
import edu.wpi.first.wpilibj.Joystick;

public class Toggle {
    Joystick joystick;
    int button;
    boolean previous;
    public Toggle (Joystick joystick, int button) {
        this.joystick = joystick;
        this.button = button;
        this.previous = false;
    }
}
