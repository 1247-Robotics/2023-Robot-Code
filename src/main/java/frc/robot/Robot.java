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
import com.revrobotics.CANSparkMax;

// import definitions.java
import frc.robot.Definitions;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // define the spark maxes
  private final CANSparkMax m_leftMaster = new CANSparkMax(Definitions.m_leftMaster, CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_leftSlave = new CANSparkMax(Definitions.m_leftSlave, CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_rightMaster = new CANSparkMax(Definitions.m_rightMaster, CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_rightSlave = new CANSparkMax(Definitions.m_rightSlave, CANSparkMax.MotorType.kBrushless);

  // define the joysticks
  private final Joystick m_joystick = new Joystick(Definitions.c_joystick);

  // define the differential drive
  private final DifferentialDrive d_drive = new DifferentialDrive(m_leftMaster, m_rightMaster);

  public boolean prevTrigger = false;
  public boolean trigger = false;
  public boolean estop = false;
  public boolean pushToStop = false;
  public boolean prevPTS = false;
  public boolean keepStopped = false;
  public boolean prevTurnMode = false;
  public boolean turnMode = false;
  public boolean invTurning = false;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // set the slave motors to follow the master motors
    m_leftSlave.follow(m_leftMaster);
    m_rightSlave.follow(m_rightMaster);

    // set the right motors to invert
    m_rightMaster.setInverted(true);
    m_leftMaster.setInverted(false);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

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
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
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
    estop = false;
    keepStopped = false;
    prevPTS = false;
    // trigger = false;
    prevTrigger = false;
    // pushToStop = false;

    // send the status of invTurning to the driver station
    DriverStation.reportWarning("invTurning: " + invTurning, false);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // get the joystick values in separate variables
    double driveX = -m_joystick.getX();
    double driveY = -m_joystick.getY();

    // round the joystick values to 2 decimal places
    driveX = Math.round(driveX * 100.0) / 100.0;
    driveY = Math.round(driveY * 100.0) / 100.0;

    // get the value of the trigger
    trigger = m_joystick.getRawButton(1);
    pushToStop = m_joystick.getRawButton(2);
    turnMode = m_joystick.getRawButton(12);

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
    double axis3 = (-m_joystick.getRawAxis(3)+1)/2;

    // drive the values between -1 and 1 logarithmically
    // x = Math.log(x) / Math.log(2);
    // y = Math.log(y) / Math.log(2);

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

    // drive the robot
    d_drive.arcadeDrive(driveY*axis3, driveX*axis3);

    // send data to shuffleboard
    SmartDashboard.putBoolean("E-Stop", estop);

    SmartDashboard.putData("Drivetrain", d_drive);

    // SmartDashboard.updateValues();

    // get the value of the trigger (button 1)
    prevTrigger = m_joystick.getRawButton(1);
    prevTurnMode = m_joystick.getRawButton(12);

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
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
