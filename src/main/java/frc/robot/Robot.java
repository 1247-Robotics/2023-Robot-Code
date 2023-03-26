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
import edu.wpi.first.wpilibj.PS4Controller;

// import com.revrobotics.CANEncoder;
// import com.ctre.phoenix.motorcontrol.DemandType;
// import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
// import edu.wpi.first.wpilibj.PS4Controller;
// import the can talon srx
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// import java.sql.Driver;
import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

// import com.revrobotics.CANSparkMax.IdleMode;
// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.cscore.HttpCamera;
// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Claw_Servo;


// import edu.wpi.first.wpilibj.controller.PIDController;
// import edu.wpi.first.wpilibj.controller.PIDSourceType;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public class startCompetition {}

  public class SensorConstants {
    public static final int HMC5883L_ADDRESS = 0x1E; // idfk what this is for
    public static final int HMC5883L_CONFIG_REG_A = 0x00;
    public static final int HMC5883L_CONFIG_REG_B = 0x01;
    public static final int HMC5883L_MODE_REG = 0x02;
    public static final int HMC5883L_MEASURE_CONT = 0x00;
    public static final int HMC5883L_MEASURE_SINGLE = 0x01;
    public static final int HMC5883L_MEASURE_IDLE = 0x03;
    public static final int HMC5883L_DATA = 0x03;
    public static final int HMC5883L_M_TO_READ = 6; // 2 bytes for each axis x, y, z
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
    
    
  }


  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "Balance";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private static final int Stage1 = 1;
  private static final int Stage2 = 2;
  private static final int Stage3 = 3;
  private static final int Stage4 = 4;
  private int Stage;


  // define the spark maxes
  private final CANSparkMax m_leftMaster = new CANSparkMax(Definitions.m_leftMaster, CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_leftSlave = new CANSparkMax(Definitions.m_leftSlave, CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_rightMaster = new CANSparkMax(Definitions.m_rightMaster, CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_rightSlave = new CANSparkMax(Definitions.m_rightSlave, CANSparkMax.MotorType.kBrushless);

  // private final CANSparkMax spinny = new CANSparkMax(Definitions.spinny, CANSparkMax.MotorType.kBrushed);
  // // setup talon srx
  private final CANSparkMax uppy = new CANSparkMax(Definitions.uppy, CANSparkMax.MotorType.kBrushless);

  // define the joysticks
  private final Joystick c_joystick = new Joystick(Definitions.c_joystick);
  //private final Joystick armJoystick = new Joystick(Definitions.armJoystick);
  private final PS4Controller c_ps4 = new PS4Controller(Definitions.c_ps4);

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

  public boolean clawClosed = false;
  public double clawAngleMax = 180;
  public double clawAngleMin = 90;
  public double clawAngle = 0;
  public double clawCloseMult = 10;


  //private DigitalInput baseLimit;
  private DigitalInput elevLimitB;
  private DigitalInput elevLimitT;
  private DigitalInput elbowLimit;

  private CANSparkMax elbowMotor = new CANSparkMax(Definitions.armPivot, CANSparkMax.MotorType.kBrushless);
  private CANSparkMax wristMotor = new CANSparkMax(Definitions.clawPivot, CANSparkMax.MotorType.kBrushless);

  double elbowPos;
  double wristPos;
  double uppyPos;

  private Claw_Servo cl_servoR = new Claw_Servo(Definitions.clawServo1);
  private Claw_Servo cl_servoL = new Claw_Servo(Definitions.clawServo2);

  private int i = 0;

  private HttpCamera limelightFeed;

  private int cyclesPerSecond = 50;

  Timer timer = new Timer();

  @Override
  public void robotInit() {

    cl_servoR.setClosed(Definitions.servo1Close);
    cl_servoL.setClosed(Definitions.servo2Close);

    cl_servoR.setOpen(Definitions.servo1Open);
    cl_servoL.setOpen(Definitions.servo2Open);
    // Create an instance of the I2C class for the gyroscope
    // I2C gyro = new I2C(I2C.Port.kOnboard, SensorConstants.ITG3205_ADDRESS);

    // // Define the register address to read from
    // byte register = SensorConstants.ITG3205_DATA;

    // // Create a buffer to hold the data read from the gyroscope
    // byte[] buffer = new byte[SensorConstants.ITG3205_G_TO_READ];

    // // Write to the register to initiate a read of the gyroscope data
    // gyro.write(register, SensorConstants.ITG3205_G_TO_READ);

    // // Read the data from the gyroscope
    // gyro.read(register, SensorConstants.ITG3205_G_TO_READ, buffer);


    // // Parse the data for each axis (x, y, z) from the buffer
    // int x = (buffer[0] << 8) | buffer[1];
    // int y = (buffer[2] << 8) | buffer[3];
    // int z = (buffer[4] << 8) | buffer[5];

    limelightFeed = new HttpCamera("limelight", "http://limelight.local:5800/stream.mjpg");
    // driverShuffleboardTab.add("LL", limelightFeed).withPosition(0, 0).withSize(15, 8).withProperties(Map.of("Show Crosshair", true, "Show Controls", false));
    
    // define the limiters
    elbowLimit = new DigitalInput(Definitions.elbowLimit);
    elevLimitB = new DigitalInput(Definitions.elevLimitB);
    elevLimitT = new DigitalInput(Definitions.elevLimitT);

    // create the autonomous modes in Shuffleboard
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("Balance", kCustomAuto);
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
    servo1 = new Servo(Definitions.clawServo1);
    servo2 = new Servo(Definitions.clawServo2);

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
    // gets the current from the master motors
    leftPower = m_leftMaster.getOutputCurrent();
    rightPower = m_rightMaster.getOutputCurrent();

    turnMode = c_joystick.getRawButton(12);

    SmartDashboard.putNumber("MATCH TIME", DriverStation.getMatchTime());
    SmartDashboard.putNumber("MATCH NUMBER", DriverStation.getMatchNumber());

    

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
    //f (turnMode == true && prevTurnMode == false) { invTurning = !invTurning; }

    // stores the current status of the turn mode button
    //prevTurnMode = c_joystick.getRawButton(12);

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
    m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    servo1.setAngle(90);
    servo2.setAngle(90);

    timer.reset();
    timer.start();

    i = 0;
    Stage = 1;
  }






  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    double timeElapsed = timer.get();
    // if (i < 100) {
    //   d_drive.arcadeDrive(0.5, 0);
    //   i++;
    //   } else {
    //     d_drive.arcadeDrive(0, 0);
    // }
    
    // switch (m_autoSelected) {
    //   case kCustomAuto:
        
    //   break;
    //   case kDefaultAuto:
    //   default:
    d_drive.arcadeDrive(0, 0);
    switch (Stage) {

      case Stage1:
        if (timeElapsed <= 1) {
          cl_servoR.close();
          cl_servoL.close();
          System.out.println("Autonomous Stage 1");
        } else { Stage++; timer.reset(); }
        break;

      case Stage2:
        if (timeElapsed <= 2) {
          elbowMotor.set(0.2);
          System.out.println("Autonomous Stage 2");
          // Stage++;
        } else { Stage++; timer.reset(); }
        break;

      case Stage3:
        if (timeElapsed <= 2) {
          servo1.setAngle(90);
          servo2.setAngle(90);
          System.out.println("Autonomous Stage 3");
        } else { Stage++; timer.reset(); }
        break;

      case Stage4:
        if (timeElapsed <= 4) {
          if (timeElapsed < 2) { elbowMotor.set(0.1); }
          d_drive.arcadeDrive(-0.7, 0);
          System.out.println("Autonomous Stage 4");
        } else { Stage++; timer.reset(); }
        break;

      default:
    }
        // // Put default auto code here
        // if (i < (2.5*cyclesPerSecond)) {
        //   d_drive.arcadeDrive(-1, 0);
        //   i++;
        // } else {
        //   d_drive.arcadeDrive(0, 0);
        // }
        // break;
        
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}
  
  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //------Elevator-----
    System.out.println(!elevLimitB.get() +", "+  c_ps4.getL1Button());
    if (!elevLimitT.get() && c_ps4.getR1Button()){
      uppy.set(-.25);
    } else if(!elevLimitB.get() && c_ps4.getL1Button()){
      uppy.set(.25);
    }

    //-----Elbow-----
    //System.out.println("Left Y Val: " + c_ps4.getLeftY());
    System.out.println(elbowLimit.get());
    if (!elbowLimit.get() && c_ps4.getLeftY() < -.1){
      elbowMotor.set(c_ps4.getLeftY()*.25);
    } else if (c_ps4.getLeftY() > .1){
      elbowMotor.set(c_ps4.getLeftY()*.125);
    } else if (!elbowLimit.get()) {
      elbowMotor.set(-.05);
      //elbowMotor.set(0);
    } else {
      elbowMotor.set(0);
    }

    //-----Wrist----
    
    if (Math.abs(c_ps4.getRightY()) > .1){
      wristMotor.set(c_ps4.getRightY()*.15);
    } else {
      wristMotor.set(-.03);
    }

    //-----Claw-----
    
    //System.out.println("Claw Open Button: " +  c_ps4.getR2Button());
    if (c_ps4.getL2ButtonPressed()){
      // Close
      servo1.setAngle(150);
      servo2.setAngle(20);
    } else if (c_ps4.getR2Button()){
      // Open
      servo1.setAngle(90);
      servo2.setAngle(90);
    }
    driveX = -c_joystick.getX();
    driveY = -c_joystick.getY();
    driveZ = -c_joystick.getZ();
    // axis3 = (-c_joystick.getRawAxis(3)+2/2);

    if (Math.abs(driveX) < Definitions.c_joystick_deadzone)   { driveX = 0; }
    if (Math.abs(driveY) < Definitions.c_joystick_deadzone)   { driveY = 0; }
    if (Math.abs(driveZ) < Definitions.c_joystick_deadzone+0.05)   { driveZ = 0; }   
    
    turnMode = c_joystick.getRawButton(12);

    driveX = driveZ*0.75;

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

  int servo1Pos = 90;
  int servo2Pos = 90;

  
  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    servo1Pos = 90;
    servo2Pos = 90;

    servo1.setAngle(servo1Pos);
    servo2.setAngle(servo2Pos);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    if (c_ps4.getTriangleButtonPressed()) {
      servo1Pos += 5;
      servo1.setAngle(servo1Pos);
      System.out.println(servo1Pos);
    }
    if (c_ps4.getCircleButtonPressed()) {
      servo2Pos += 5;
      servo2.setAngle(servo2Pos);
      System.out.println(servo2Pos);
    }
    if (c_ps4.getSquareButtonPressed()) {
      servo2Pos -= 5;
      servo2.setAngle(servo2Pos);
      System.out.println(servo2Pos);
    }
    if (c_ps4.getCrossButtonPressed()) {
      servo1Pos -= 5;
      servo1.setAngle(servo1Pos);
      System.out.println(servo1Pos);
    }
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
