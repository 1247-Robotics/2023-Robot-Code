// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.interfaces.Accelerometer;
// import edu.wpi.first.wpilibj.motorcontrol.Talon;
// import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.BuiltInAccelerometer;
// import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;

// import com.ctre.phoenix.motorcontrol.DemandType;
// import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
// import edu.wpi.first.wpilibj.PS4Controller;
// import the can talon srx
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// import java.sql.Driver;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

// import com.revrobotics.CANSparkMax.IdleMode;
// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Servo;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.DigitalInput;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public class startCompetition {}

  public class SensorConstants {
    public static final int BMP085_ADDRESS = 0x77; //address of the Barometer
    
    public static final int BMA180_ADDRESS = 0x40; //address of the accelerometer
    public static final int BMA180_RESET = 0x10;
    public static final int BMA180_PWR = 0x0D;
    public static final int BMA180_BW = 0x20;
    public static final int BMA180_RANGE = 0x35;
    public static final int BMA180_DATA = 0x02;
    public static final int BMA180_A_TO_READ = 6; // 2 bytes for each axis x, y, z
    
    public static final int ITG3205_ADDRESS = 0x68; //address of the Gyroscope
    public static final int ITG3205_SMPLRT_DIV = 0x15;
    public static final int ITG3205_DLPF_FS = 0x16;
    public static final int ITG3205_INT_CFG = 0x17;
    public static final int ITG3205_PWR_MGM = 0x3E;
    public static final int ITG3205_DATA = 0x1B;
    public static final int ITG3205_G_TO_READ = 8; // 2 bytes for each axis x, y, z, temp
    
    public static final int HMC5883L_ADDRESS = 0x1E;
    public static final int HMC5883L_CONFIG_REG_A = 0x00;
    public static final int HMC5883L_CONFIG_REG_B = 0x01;
    public static final int HMC5883L_MODE_REG = 0x02;
    public static final int HMC5883L_MEASURE_CONT = 0x00;
    public static final int HMC5883L_MEASURE_SINGLE = 0x01;
    public static final int HMC5883L_MEASURE_IDLE = 0x03;
    public static final int HMC5883L_DATA = 0x03;
    public static final int HMC5883L_M_TO_READ = 6; // 2 bytes for each axis x, y, z
  }
  
  // Create an instance of the I2C class for the device you want to communicate with
  I2C device = new I2C(I2C.Port.kOnboard, SensorConstants.BMA180_ADDRESS);

  // Define the register address to read from
  byte register = 0x02;

  // Create a buffer to hold the data read from the device
  byte[] buffer = new byte[2];

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "thing";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // define the spark maxes
  private final CANSparkMax m_leftMaster = new CANSparkMax(Definitions.m_leftMaster, CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_leftSlave = new CANSparkMax(Definitions.m_leftSlave, CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_rightMaster = new CANSparkMax(Definitions.m_rightMaster, CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_rightSlave = new CANSparkMax(Definitions.m_rightSlave, CANSparkMax.MotorType.kBrushless);

  // private final CANSparkMax spinny = new CANSparkMax(Definitions.spinny, CANSparkMax.MotorType.kBrushed);
  // // setup talon srx
  // private final CANSparkMax uppy = new CANSparkMax(Definitions.uppy, CANSparkMax.MotorType.kBrushless);

  // define the joysticks
  private final Joystick c_joystick = new Joystick(Definitions.c_joystick);
  private final Joystick armJoystick = new Joystick(Definitions.armJoystick);
  // private final PS4Controller c_ps4 = new PS4Controller(Definitions.c_ps4);

  // define the differential drive
  public final DifferentialDrive d_drive = new DifferentialDrive(m_leftMaster, m_rightMaster);

  public Servo servo1;
  public Servo servo2;

  public boolean prevTurnMode = false;
  public boolean turnMode = false;
  public boolean invTurning = false;
  public boolean turning = false;
  public boolean turnLeft = false;
  public boolean turnRight = false;
  public boolean throttleLever0 = false;

  public double lastAccelY = 0;
  public double lastDriveY = 0;

  public boolean impact = false;

  public int cycle = 0;
  public int lastCycle = 0;

  public int watch = 0;

  public double leftPower = m_leftMaster.getOutputCurrent();
  public double rightPower = m_rightMaster.getOutputCurrent();

  public double driveX = -c_joystick.getX();
  public double driveY = -c_joystick.getY();
  public double driveZ = -c_joystick.getZ();
  // public double axis3 = (-c_joystick.getRawAxis(3)+1)/2;

  public double armX;
  public double armY;
  public double arm3;

  public double accelX = 0;
  public double accelY = 0;
  public double accelZ = 0;

  public int loop = 0;
  public boolean loopToggle = false;
  public boolean move = false;

  public boolean prev10 = false;

  public double angle = armJoystick.getZ();

  public boolean prevClawTrig = armJoystick.getRawButton(1);

  public boolean clawTrig = armJoystick.getRawButton(1);

  public boolean clawClosed = false;

  private DigitalInput baseLimit;
  private DigitalInput elevLimitB;
  private DigitalInput elevLimitT;

  // private int elevStatus;

  // private int wasRotate = 0;

  // private boolean prevSwitchButton = armJoystick.getRawButton(2);

  // private int activeMotors = 0;
  // private double elev;
  // private double pivot;
  // private double elbow;
  // private double wrist;

  // private CANSparkMax elbowMotor = new CANSparkMax(Definitions.armPivot, CANSparkMax.MotorType.kBrushless);
  // private TalonSRX wristMotor = new TalonSRX(Definitions.clawPivot);

  private int i = 0;
  

  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    
    // define the limiters
    baseLimit = new DigitalInput(Definitions.baseLimit);
    elevLimitB = new DigitalInput(Definitions.elevLimitB);
    elevLimitT = new DigitalInput(Definitions.elevLimitT);

    // create the autonomous modes in Shuffleboard
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("thing", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // set the slave motors to follow the master motors
    m_leftSlave.follow(m_leftMaster);
    m_rightSlave.follow(m_rightMaster);

    // set the right motors to invert
    m_rightMaster.setInverted(true);
    m_leftMaster.setInverted(false);

    // starts the camera server
    // CameraServer.startAutomaticCapture();

    // ensure the Spark Maxes are set to Brake
    m_leftMaster.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_rightMaster.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_rightSlave.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_leftSlave.setIdleMode(CANSparkMax.IdleMode.kBrake);

    // define the claw servos
    // servo1 = new Servo(Definitions.clawServo1);
    // servo2 = new Servo(Definitions.clawServo2);

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Read data from the device
    // device.read(register, buffer.length, buffer);

    // Convert the buffer to a numerical value
    // int numericalValue = (buffer[0] << 8) | buffer[1];

    // Convert the numerical value to a double
    // double value = Double.valueOf(numericalValue);
    // SmartDashboard.putNumber("", value);

    // gets the current from the master motors
    leftPower = m_leftMaster.getOutputCurrent();
    rightPower = m_rightMaster.getOutputCurrent();

    turnMode = c_joystick.getRawButton(12);

    

    // driveZ /= axis3*1.6;/

    // makes the z axis usable as a low speed turn
    if (driveX < 0.2 && driveX > -0.2 && driveZ != 0)   { driveX = driveX+(driveZ*0.4); }
    
    // constantly refreshes the shuffleboard data
    SmartDashboard.putNumber("Left Power", leftPower);
    SmartDashboard.putNumber("Right Power", rightPower);
    SmartDashboard.putBoolean("Invert X on -Y", invTurning);
    SmartDashboard.putData("Drivetrain", d_drive);
    SmartDashboard.putBoolean("Turning", turning);
    SmartDashboard.putBoolean("Turn Left", turnLeft);
    SmartDashboard.putBoolean("Turn Right", turnRight);
    SmartDashboard.putBoolean("Throttle lever 0", throttleLever0);
    SmartDashboard.putData("Drivetrain", d_drive);
    // SmartDashboard.putBoolean("Top limiter", elevLimitT.get());
    // SmartDashboard.putBoolean("Bottom Limiter", elevLimitB.get());
    // SmartDashboard.putBoolean("Base limiter", baseLimit.get());

    // pulls data from limelight
    // double targetValid = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    // double targetArea = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

    // displays an indicator when the speed multiplier is too low
    // if (axis3 <= 0.12)
    // {
    //   throttleLever0 = true;
    // } else
    // {
    //   throttleLever0 = false;
    // }

    // toggles the turning inversion
    if (turnMode == true && prevTurnMode == false) { invTurning = !invTurning; }

    // stores the current status of the turn mode button
    prevTurnMode = c_joystick.getRawButton(12);

    // if (axis3 <= 0.12)    { throttleLever0 = true; }
    // else    { throttleLever0 = false; }
  }





  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    i = 0;
  }






  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if (i < 100) {
      d_drive.arcadeDrive(0.5, 0);
      i++;
      } else {
        d_drive.arcadeDrive(0, 0);
    }
    
    // switch (m_autoSelected) {
    //   case kCustomAuto:
    //     if (i < 100) {
    //     d_drive.arcadeDrive(0.5, 0);
    //     i++;
    //     } else {
    //       d_drive.arcadeDrive(0, 0);
    //     }
    //   break;
    //   case kDefaultAuto:
    //   default:
    //     // Put default auto code here
    //     if (i < 100) {
    //       d_drive.arcadeDrive(0.5, 0);
    //       i++;
    //     } else {
    //       d_drive.arcadeDrive(0, 0);
    //     }
    //     break;
        
    // }
  }






  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

    // activeMotors = 0;
    
    

    // send the status of invTurning to the driver station
    DriverStation.reportWarning("invTurning: " + invTurning, false);
  }





  
  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    // if (armJoystick.getRawButton(2) && !prevSwitchButton) {
    //   if (activeMotors == 0) {
    //     activeMotors = 1;
    //   } else if (activeMotors == 1) {
    //     activeMotors = 0;
    //   }
    // }

    // prevSwitchButton = armJoystick.getRawButton(2);

    // if (armJoystick.getRawButton(1) && !prevClawTrig) {
    //   clawClosed = !clawClosed;
    // }

    // prevClawTrig = armJoystick.getRawButton(1);

    // if (clawClosed) {
    //   angle = 270;
    // } else {
    //   angle = 0;
    // }

    // servo1.setAngle(angle);
    // if (angle == 270) {
    //   servo2.setAngle(0);
    // } else {
    //   servo2.setAngle(270);
    // }
    
    
    // armX = -armJoystick.getX();
    // armY = -armJoystick.getY();
    // arm3 = ((-armJoystick.getZ())+1)/2;

    // // adds a deadzone
    // if (Math.abs(armX) < Definitions.c_joystick_deadzone)   { driveX = 0; }
    // if (Math.abs(armY) < Definitions.c_joystick_deadzone)   { driveY = 0; }
    // if (Math.abs(arm3) < Definitions.c_joystick_deadzone)   { driveZ = 0; }

    // if (activeMotors == 0) {
    //   elev = armY;
    //   pivot = armX;
    //   wrist = 0;
    //   elbow = 0;
    // } else if (activeMotors == 1) {
    //   elev = 0;
    //   pivot = 0;
    //   wrist = armX;
    //   elbow = armY;
    // }
    // if (Math.abs(elev)>0.05) {
    //       elev = 0.1;
    //     }

    // if (elevLimitT.get()) {
    //   elev = Math.max(armY, 0);
    // } else if (elevLimitB.get()) {
    //   elev = -Math.max(-armY, 0);
    // }

    // if (baseLimit.get()) {
    //   if (pivot > 0 || wasRotate == 1) {
    //     wasRotate = 1;
    //     pivot = -Math.max(-armX, 0);
    //   } else if (armX < 0 || wasRotate == -1) {
    //     wasRotate = -1;
    //     pivot = Math.max(armX, 0);
    //   } else {
    //     DriverStation.reportError("Base limiter hit but side cannot be determined!", false);
    //   }
    // } else {
    //   wasRotate = 0;
    // }

    

    // send armX and armY to spinny
    // spinny.set(pivot*arm3*0.8);
    // d_drive.arcadeDrive(0, 0);
    // uppy.set(elev*0.8);
    // elbowMotor.set(elbow*0.5);
    // wristMotor.set(TalonSRXControlMode.Current, wrist);


    driveX = -c_joystick.getX();
    driveY = -c_joystick.getY();
    driveZ = -c_joystick.getZ();
    // axis3 = (-c_joystick.getRawAxis(3)+2/2);

    if (Math.abs(driveX) < Definitions.c_joystick_deadzone)   { driveX = 0; }
    if (Math.abs(driveY) < Definitions.c_joystick_deadzone)   { driveY = 0; }
    if (Math.abs(driveZ) < Definitions.c_joystick_deadzone+0.05)   { driveZ = 0; }   
    
    turnMode = c_joystick.getRawButton(12);
    // if Math.abs(driveX) izsaq1sn't more than 0.2 then driveZ can be used as steering
    // if (Math.abs(driveX) < 0.2 && driveZ != 0)   { driveX = driveX+(driveZ*0.4); }
    driveX = driveZ*0.75;
    // driveX = Math.log(driveX);

    // SmartDashboard.putNumber("Left Power", leftPower);
    // SmartDashboard.putNumber("Right Power", rightPower);
    // SmartDashboard.putBoolean("E-Stop", estop);
    // SmartDashboard.putBoolean("Invert X on -Y", invTurning);
    // SmartDashboard.putData("Drivetrain", d_drive);
    // SmartDashboard.putBoolean("Turning", turning);
    // SmartDashboard.putBoolean("Turn Left", turnLeft);
    // SmartDashboard.putBoolean("Turn Right", turnRight);
    // SmartDashboard.putBoolean("Throttle lever 0", throttleLever0);
    // SmartDashboard.putBoolean("E-Stop", estop);
    // SmartDashboard.putData("Drivetrain", d_drive);

    if (turnMode == true && prevTurnMode == false) { invTurning = !invTurning; }

    prevTurnMode = c_joystick.getRawButton(12);

    if (driveX != 0) {
      turning = true;
      if (driveX > 0) {
        turnLeft = true;
        turnRight = false;
      } else if (driveX < 0) {
        turnLeft = false;
        turnRight = true;
      }
    } else {
      turning = false;
      turnLeft = false;
      turnRight = false;
    }

    // if turning is set to invert, invert the driveX value when the driveY value is less than 0
    if (invTurning) { driveX = -driveX; }

    // drive the robot
    d_drive.arcadeDrive(driveY, driveX);
    
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    // send 0 to the motors
    d_drive.arcadeDrive(0, 0);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    // get the speed from the encoders
    double leftSpeed = m_leftMaster.getEncoder().getVelocity();
    double rightSpeed = m_rightMaster.getEncoder().getVelocity();

    m_leftSlave.setIdleMode(IdleMode.kCoast);

    

    // if the speed is greater than 0.1 for either motor send an alert to driver station
    if (leftSpeed > 0.1 || rightSpeed > 0.1) {
      DriverStation.reportError("Motors are still moving!", false);
    }
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
