// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Claw_Servo;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto  = "Balance";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private static final int Stage1 = 1;
  private static final int Stage2 = 2;
  private static final int Stage3 = 3;
  private static final int Stage4 = 4;
  private static final int Stage5 = 5;
  private int Stage;


  // define the spark maxes
  private final CANSparkMax m_leftMaster  = new CANSparkMax(Definitions.m_leftMaster, CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_leftSlave   = new CANSparkMax(Definitions.m_leftSlave, CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_rightMaster = new CANSparkMax(Definitions.m_rightMaster, CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_rightSlave  = new CANSparkMax(Definitions.m_rightSlave, CANSparkMax.MotorType.kBrushless);
  // private final CANSparkMax uppy          = new CANSparkMax(Definitions.uppy, CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax elbowMotor    = new CANSparkMax(Definitions.armPivot, CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax wristMotor    = new CANSparkMax(Definitions.clawPivot, CANSparkMax.MotorType.kBrushless);

  // define the joysticks
  private final Joystick c_joystick = new Joystick(Definitions.c_joystick);
  private final PS4Controller c_ps4 = new PS4Controller(Definitions.c_ps4);

  // define the differential drive
  public final DifferentialDrive d_drive = new DifferentialDrive(m_leftMaster, m_rightMaster);

  // public Servo servo1;
  // public Servo servo2;

  public boolean prevTurnMode   = false;
  public boolean turnMode       = false;
  public boolean invTurning     = false;
  public boolean turning        = false;
  public boolean turnLeft       = false;
  public boolean turnRight      = false;
  public boolean throttleLever0 = false;

  public double leftPower  = m_leftMaster.getOutputCurrent();
  public double rightPower = m_rightMaster.getOutputCurrent();

  public double driveX = -c_joystick.getX();
  public double driveY = -c_joystick.getY();
  public double driveZ = -c_joystick.getZ();

  //private DigitalInput baseLimit;
  private DigitalInput elevLimitB;
  private DigitalInput elevLimitT;
  private DigitalInput elbowLimit;

  private Claw_Servo cl_servoR = new Claw_Servo(Definitions.clawServo1);
  private Claw_Servo cl_servoL = new Claw_Servo(Definitions.clawServo2);

  private HttpCamera limelightFeed;

  Timer timer = new Timer();

  private boolean ignoreLimits = false;

  @Override
  public void robotInit() {

    cl_servoR.setClosed(Definitions.servo1Close);
    cl_servoL.setClosed(Definitions.servo2Close);

    cl_servoR.setOpen(Definitions.servo1Open);
    cl_servoL.setOpen(Definitions.servo2Open);

    limelightFeed = new HttpCamera("limelight", "http://limelight.local:5800/stream.mjpg");

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
    SmartDashboard.putData("Drivetrain", d_drive);

    SmartDashboard.putNumber("Arm", elbowMotor.get());
    SmartDashboard.putNumber("Wrist", wristMotor.get());
    // SmartDashboard.putNumber("Elevator", uppy.get());
    SmartDashboard.putNumber("Left Claw Servo", cl_servoL.current());
    SmartDashboard.putNumber("Right Claw Servo", cl_servoR.current());

    SmartDashboard.putBoolean("Ignore limiters", ignoreLimits);
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

    cl_servoL.open();
    cl_servoR.open();

    timer.reset();
    timer.start();

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
        if (timeElapsed <= .2) {
          cl_servoR.close();
          cl_servoL.close();
          System.out.println("Autonomous Stage 1");
        } else { Stage++; timer.reset(); }
        break;

      case Stage2:
        if (timeElapsed <= 1) {
          elbowMotor.set(0.15);
          System.out.println("Autonomous Stage 2");
          // Stage++;
        } else { Stage++; timer.reset(); }
        break;

      case Stage3:
        if (timeElapsed <= .3) {
          cl_servoR.open();
          cl_servoL.open();
          System.out.println("Autonomous Stage 3");
        } else { Stage++; timer.reset(); }
        break;

      case Stage4:
        if (timeElapsed <= 2) { elbowMotor.set(-0.3); }
        else { Stage++; timer.reset(); }
        break;
      case Stage5:
          if (timeElapsed <= 2.5) {
          d_drive.arcadeDrive(-0.6, 0);
          System.out.println("Autonomous Stage 5");
        } else { Stage++; timer.reset(); }
        break;

      default:
    }        
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}
  
  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (c_joystick.getRawButton(2) && c_joystick.getRawButton(1)) { ignoreLimits = !ignoreLimits; }

    if (c_ps4.getTriangleButton() && c_ps4.getSquareButton()) { ignoreLimits = !ignoreLimits; }

    if (!ignoreLimits) {
      // //------Elevator-----
      // System.out.println(!elevLimitB.get() +", "+  c_ps4.getL1Button());
      // if (!elevLimitT.get() && c_ps4.getR1Button()){
      //   uppy.set(-.25);
      // } else if(!elevLimitB.get() && c_ps4.getL1Button()){
      //   uppy.set(.25);
      // }

      //-----Elbow-----
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
    }
    else
    {
      // // Elevator
      // if      (c_ps4.getR1Button()) { uppy.set(-.25); }
      // else if (c_ps4.getL1Button()) { uppy.set(.25); }

      // Elbow
      if      (c_ps4.getLeftY() < -.1) { elbowMotor.set(c_ps4.getLeftY()*.25); }
      else if (c_ps4.getLeftY() > .1) { elbowMotor.set(c_ps4.getLeftY()*.125); }
      else    { elbowMotor.set(-.05); }
    }

    //-----Wrist----
    
    if   (Math.abs(c_ps4.getRightY()) > .1){ wristMotor.set(c_ps4.getRightY()*.15); }
    else { wristMotor.set(-.03); }

    //-----Claw-----
    
    //System.out.println("Claw Open Button: " +  c_ps4.getR2Button());
    if (c_ps4.getL2ButtonPressed() || c_ps4.getR2ButtonPressed()){
      if (cl_servoL.isClosed()) {
        // Close
        cl_servoR.open();
        cl_servoL.open();
      } else {
        cl_servoR.close();
        cl_servoL.close();
      }
    }

    driveX = -c_joystick.getX();
    driveY = -c_joystick.getY();
    driveZ = -c_joystick.getZ();

    if (Math.abs(driveX) < Definitions.c_joystick_deadzone)   { driveX = 0; }
    if (Math.abs(driveY) < Definitions.c_joystick_deadzone)   { driveY = 0; }
    if (Math.abs(driveZ) < Definitions.c_joystick_deadzone+0.05)   { driveZ = 0; }   
    
    turnMode = c_joystick.getRawButton(12);

    driveX = driveZ*0.75+driveX*0.4;

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
    if (invTurning) { driveY = -driveY; }

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

    cl_servoR.set(servo1Pos);
    cl_servoL.set(servo2Pos);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    SmartDashboard.putNumber("Left Claw Servo", cl_servoL.current());
    SmartDashboard.putNumber("Right Claw Servo", cl_servoR.current());
    if (c_ps4.getTriangleButtonPressed()) {
      cl_servoR.set(cl_servoR.getLastManual() + 5);
      System.out.println(cl_servoR.getLastManual());
    }
    if (c_ps4.getCircleButtonPressed()) {
      cl_servoL.set(cl_servoL.getLastManual() + 5);
      System.out.println(cl_servoL.getLastManual());
    }
    if (c_ps4.getSquareButtonPressed()) {
      cl_servoL.set(cl_servoL.getLastManual() - 5);
      System.out.println(cl_servoL.getLastManual());
    }
    if (c_ps4.getCrossButtonPressed()) {
      cl_servoR.set(cl_servoR.getLastManual() - 5);
      System.out.println(cl_servoR.getLastManual());
    }
    if (c_ps4.getL1ButtonPressed()) {
      if (cl_servoL.isOpen()) {
        cl_servoL.close();
      } else { cl_servoL.open(); }
      }
    if (c_ps4.getR1ButtonPressed()) {
      if (cl_servoR.isOpen()) {
        cl_servoR.close();
      } else { cl_servoR.open(); }
    }
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
