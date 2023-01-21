// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
// import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
// import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.PS4Controller;

// import java.sql.Driver;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.cameraserver.CameraServer;

// import definitions.java
// import frc.robot.Definitions;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "Run motors (stop if movement)";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // define the spark maxes
  private final CANSparkMax m_leftMaster = new CANSparkMax(Definitions.m_leftMaster, CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_leftSlave = new CANSparkMax(Definitions.m_leftSlave, CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_rightMaster = new CANSparkMax(Definitions.m_rightMaster, CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_rightSlave = new CANSparkMax(Definitions.m_rightSlave, CANSparkMax.MotorType.kBrushless);

  // define the joysticks
  private final Joystick c_joystick = new Joystick(Definitions.c_joystick);
  // private final PS4Controller c_ps4 = new PS4Controller(Definitions.c_ps4);

  // define the differential drive
  public final DifferentialDrive d_drive = new DifferentialDrive(m_leftMaster, m_rightMaster);

  public boolean prevTrigger = false;
  public boolean trigger = false;
  public boolean estop = false;
  public boolean pushToStop = false;
  public boolean prevPTS = false;
  public boolean keepStopped = false;
  public boolean prevTurnMode = false;
  public boolean turnMode = false;
  public boolean invTurning = false;
  public boolean turning = false;
  public boolean turnLeft = false;
  public boolean turnRight = false;
  public boolean throttleLever0 = false;
  public boolean kill = false;

  // get the roboeio's internal accelerometer
  Accelerometer internalAccel = new BuiltInAccelerometer();

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
  public double axis3 = (-c_joystick.getRawAxis(3)+1)/2;

  public double accelX = 0;
  public double accelY = 0;
  public double accelZ = 0;

  public int loop = 0;
  public boolean loopToggle = false;
  public boolean move = false;

  public boolean prev10 = false;

  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("Run motors (stop if movement)", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // set the slave motors to follow the master motors
    m_leftSlave.follow(m_leftMaster);
    m_rightSlave.follow(m_rightMaster);

    // set the right motors to invert
    m_rightMaster.setInverted(true);
    m_leftMaster.setInverted(false);

    CameraServer.startAutomaticCapture();

    // create a second camera for a backup camera

    m_leftMaster.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_rightMaster.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_rightSlave.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_leftSlave.setIdleMode(CANSparkMax.IdleMode.kBrake);
    // display the mode on the smart dashboard
    SmartDashboard.putString("Motor Mode", "Brake");
    // CameraServer.


    
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
    leftPower = m_leftMaster.getOutputCurrent();
    rightPower = m_rightMaster.getOutputCurrent();

    driveX = -c_joystick.getX();
    driveY = -c_joystick.getY();
    driveZ = -c_joystick.getZ();
    axis3 = (-c_joystick.getRawAxis(3)+2/2);

    trigger = c_joystick.getRawButton(1);
    pushToStop = c_joystick.getRawButton(2);
    turnMode = c_joystick.getRawButton(12);

    if (Math.abs(driveX) < Definitions.c_joystick_deadzone)   { driveX = 0; }
    if (Math.abs(driveY) < Definitions.c_joystick_deadzone)   { driveY = 0; }
    if (Math.abs(driveZ) < Definitions.c_joystick_deadzone+0.05)   { driveZ = 0; }

    driveZ /= axis3*1.6;

    if (driveX < 0.2 && driveX > -0.2 && driveZ != 0)   { driveX = driveX+(driveZ*0.4); }

    accelX = internalAccel.getX();
    accelY = internalAccel.getY();
    accelZ = internalAccel.getZ();

    SmartDashboard.putNumber("Left Power", leftPower);
    SmartDashboard.putNumber("Right Power", rightPower);
    SmartDashboard.putBoolean("E-Stop", estop);
    SmartDashboard.putBoolean("Invert X on -Y", invTurning);
    SmartDashboard.putData("Drivetrain", d_drive);
    SmartDashboard.putBoolean("Turning", turning);
    SmartDashboard.putBoolean("Turn Left", turnLeft);
    SmartDashboard.putBoolean("Turn Right", turnRight);
    SmartDashboard.putBoolean("Throttle lever 0", throttleLever0);
    SmartDashboard.putBoolean("E-Stop", estop);
    SmartDashboard.putData("Drivetrain", d_drive);

    // a button that when pressed will toggle the motor mode between brake and coast
    if (c_joystick.getRawButton(10) && !prev10) {
      if (m_leftMaster.getIdleMode() == CANSparkMax.IdleMode.kBrake) {
        m_leftMaster.setIdleMode(CANSparkMax.IdleMode.kCoast);
        m_rightMaster.setIdleMode(CANSparkMax.IdleMode.kCoast);
        m_rightSlave.setIdleMode(CANSparkMax.IdleMode.kCoast);
        m_leftSlave.setIdleMode(CANSparkMax.IdleMode.kCoast);
        SmartDashboard.putString("Motor Mode", "Coast");
      } else {
        m_leftMaster.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_rightMaster.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_rightSlave.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_leftSlave.setIdleMode(CANSparkMax.IdleMode.kBrake);
        SmartDashboard.putString("Motor Mode", "Brake");
      }
    }

    prev10 = c_joystick.getRawButton(10);

    if (axis3 <= 0.12)    { throttleLever0 = true; }
    else    { throttleLever0 = false; }

    if (trigger == true && prevTrigger == false && keepStopped == true && pushToStop == true)    { estop = keepStopped = false; }
    else if (trigger == true && prevTrigger == false && pushToStop == true)    { estop = keepStopped = true; }
    else if (trigger == true && prevTrigger == false && impact == false) {
      estop = !estop;
      keepStopped = !keepStopped;
    }

    if (turnMode == true && prevTurnMode == false) { invTurning = !invTurning; }

    if (pushToStop == true && keepStopped == false) { estop = prevPTS = true; }
    else if (prevPTS == true && keepStopped == false) { estop = prevPTS = false; }

    prevTrigger = c_joystick.getRawButton(1);
    prevTurnMode = c_joystick.getRawButton(12);
  }

  public boolean checkMovement() {
    if (accelX >= 0.1 && accelY >= 0.1 && accelZ >= 0.1/*&& accelX <= -0.01 && accelY <= -0.01 && accelZ <= -0.01*/) {
      // d_drive.arcadeDrive(0, 0);
      DriverStation.reportError("Robot moved", false);
      return true;
    } else { return false; }

    
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
    loop = 0;
    move = false;
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        if (!move) {
          move = checkMovement();
          if (loop >= 15000) { loopToggle = !loopToggle; loop = 0; }
          if (loopToggle == true) {
            move = checkMovement();
            d_drive.arcadeDrive(-1, 0);
            move = checkMovement();
          } else {
            move = checkMovement();
            d_drive.arcadeDrive(1, 0);
            move = checkMovement();
          }
          loop += 1;
          
        } else { d_drive.arcadeDrive(0, 0); }
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    
    

    // send the status of invTurning to the driver station
    DriverStation.reportWarning("invTurning: " + invTurning, false);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
    if (driveX != 0 && estop == false) {
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

    if (estop) { driveX = 0; driveY = 0; }

    // drive the robot
    d_drive.arcadeDrive(driveY*axis3, driveX*axis3);

    // record the values for comparison with latest values next run
    
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    // send 0 to the motors
    d_drive.arcadeDrive(0, 0);
    estop = false;
    keepStopped = false;
    prevPTS = false;
    pushToStop = false;
    trigger = false;
    prevTrigger = false;
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    // get the speed from the encoders
    double leftSpeed = m_leftMaster.getEncoder().getVelocity();
    double rightSpeed = m_rightMaster.getEncoder().getVelocity();

    

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
  public void testPeriodic() {
    // get the internal accelerometer values
    // double accelX = internalAccel.getX();
    // double accelY = internalAccel.getY();
    // double accelZ = internalAccel.getZ();
    // if any of the accel values exceeds 0.1 set kill to true
    // if (accelX >= 0.1 )

    // get the joystick values in separate variables
    double driveX = -c_joystick.getX();
    double driveY = -c_joystick.getY();

    double axis3 = (-c_joystick.getRawAxis(3)+1)/2;


    // driveX = -c_ps4.getLeftX();
    // driveY = -c_ps4.getLeftY();
    // axis3 = 1;

      // use logaritmic scaling for the joystick values
    // driveX = Math.copySign(Math.pow(driveX, 2), driveX);
    // driveY = Math.copySign(Math.pow(driveY, 2), driveY);


    // round the joystick values to 2 decimal places
    driveX = Math.round(driveX * 100.0) / 100.0;
    driveY = Math.round(driveY * 100.0) / 100.0;

    // get the value of the trigger
    trigger = c_joystick.getRawButton(1);
    pushToStop = c_joystick.getRawButton(2);
    turnMode = c_joystick.getRawButton(12);

    if (trigger == true && prevTrigger == false) {
      estop = !estop;
      keepStopped = !keepStopped;
    }

    if (pushToStop == true && keepStopped == false) {
      estop = true;
      prevPTS = true;
    } else if (prevPTS == true && keepStopped == false) {
      estop = false;
      prevPTS = false;
    }

    if (turnMode == true && prevTurnMode == false) {
      invTurning = !invTurning;
    }

    // send estop to shuffleboard
    // update estop on shuffleboard
    

    // get the value of axis 3

    if (driveY < -0.1 && driveY > 0.1) {
      driveY = driveY * 0.2;
    }
    if (driveY < -0.1 && invTurning == true) {
      driveX = -driveX;
    }

    if (estop) {
      driveX = 0;
      driveY = 0;
    }

    // lower the output of the joystick to prevent the robot from putting out too much torque
    driveX *= 0.7;
    driveY *= 0.7;

    // drive the robot
    d_drive.arcadeDrive(driveY*axis3, driveX*axis3);

    // get the value of the trigger (button 1)
    prevTrigger = c_joystick.getRawButton(1);
    prevTurnMode = c_joystick.getRawButton(12);
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
