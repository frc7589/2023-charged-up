// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.*;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private XboxController driveCon;
  private XboxController takeCon;

  /*
   * I suggest moving these constant variables to `Constants.java`,
   * so you don't have to make everything CAPS. Also it will make the code less messy.
   * You should have learnt how to do that in class, you might want to do this as quickly as possible.
   */
  private static final int INTAKE_ID = 6; 
  private static final int ARM_ID = 3;

  private static final int RIGHT_FRONT_ID = 7;
  private static final int LEFT_FRONT_ID = 9;
  private static final int RIGHT_REAR_ID = 8;
  private static final int LEFT_REAR_ID = 1;

  private CANSparkMax intakeMotor;
  private CANSparkMax armMotor;

  private double intakeSpeedPull = 0.6; // intake initial speed
  private double intakeSpeedPush = 0.6;
  private double armSpeed = 0.4; // arm initial speed
  private double drivespeed = 0.5; // car initial speed

  private int pov;

  private WPI_VictorSPX motorRightRear;
  private WPI_VictorSPX motorRightFront;
  private WPI_VictorSPX motorLeftFront;
  private WPI_VictorSPX motorLeftRear;

  private MotorControllerGroup leftGroup;
  private MotorControllerGroup rightGroup;
  private DifferentialDrive drive;

  private AHRS ahrs;

  private AutonomousDrive autoCommand;

  // private boolean LeftYPushStat = false;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    driveCon = new XboxController(0);
    takeCon = new XboxController(1);

    intakeMotor = new CANSparkMax(INTAKE_ID, MotorType.kBrushless);
    armMotor = new CANSparkMax(ARM_ID, MotorType.kBrushless);

    motorRightRear = new WPI_VictorSPX(RIGHT_REAR_ID);
    motorRightFront = new WPI_VictorSPX(RIGHT_FRONT_ID);
    motorLeftFront = new WPI_VictorSPX(LEFT_FRONT_ID);
    motorLeftRear = new WPI_VictorSPX(LEFT_REAR_ID);

    leftGroup = new MotorControllerGroup(motorLeftFront, motorLeftRear);
    rightGroup = new MotorControllerGroup(motorRightFront, motorRightRear);
    drive = new DifferentialDrive(leftGroup, rightGroup);

    drive.setSafetyEnabled(false);

    leftGroup.setInverted(true);

    ahrs = new AHRS(SPI.Port.kMXP);

    /* AUTO HERE AUTO HERE AUTO HERE AUTO HERE AUTO HERE AUTO HERE */
    /* AUTO HERE AUTO HERE AUTO HERE AUTO HERE AUTO HERE AUTO HERE */
    /* AUTO HERE AUTO HERE AUTO HERE AUTO HERE AUTO HERE AUTO HERE */
    autoCommand = new AutonomousDrive(drive, armMotor, intakeMotor, ahrs, AutonomousDrive.B1, true);
    /* AUTO HERE AUTO HERE AUTO HERE AUTO HERE AUTO HERE AUTO HERE */
    /* AUTO HERE AUTO HERE AUTO HERE AUTO HERE AUTO HERE AUTO HERE */
    /* AUTO HERE AUTO HERE AUTO HERE AUTO HERE AUTO HERE AUTO HERE */

    SmartDashboard.putNumber("drive speed", 0.5);
    drivespeed = SmartDashboard.getNumber("drive speed", 0.5);
    SmartDashboard.putNumber("arm speed", 0.5);
    armSpeed = SmartDashboard.getNumber("arm speed", 0.5);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    autoCommand.initialize();
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

    autoCommand.execute();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // ########## CAR ##########
    drivespeed = SmartDashboard.getNumber("drive speed", drivespeed);
    if (driveCon.getLeftBumperPressed() && drivespeed >= 0.5) {
      drivespeed -= 0.1;
      SmartDashboard.putNumber("drive speed", drivespeed);
    } else if (driveCon.getRightBumperPressed() && drivespeed <= 0.9) {
      drivespeed += 0.1;
      SmartDashboard.putNumber("drive speed", drivespeed);
    }

    // I highly doubt it is necessary to make it drive at 0 speed when both triggers are pressed
    // Like in most games, when you press both w and s, it usually move either up or down and not stop moving 
    double driveConLeftTrigger = driveCon.getLeftTriggerAxis();
    double driveConRightTrigger = driveCon.getRightTriggerAxis();
    if (driveConLeftTrigger >= 0.1 && driveConRightTrigger >= 0.1) {
      drive.tankDrive(0, 0);
    } else if (driveConLeftTrigger >= 0.1) {
      drive.tankDrive(driveConLeftTrigger * drivespeed, driveConRightTrigger * -drivespeed);
    } else if (driveConRightTrigger >= 0.1) {
      drive.tankDrive(driveConRightTrigger * -drivespeed, driveConRightTrigger * drivespeed);
    }

    double driveConLeftY = driveCon.getLeftY();
    double driveConRightY = driveCon.getRightY();
    if (Math.abs(driveConLeftY) >= 0.1 || Math.abs(driveConRightY) >= 0.1) {
      drive.tankDrive(driveConLeftY * drivespeed, driveConRightY * drivespeed);
    } else {
      drive.tankDrive(0, 0);
    }

    /* ############# ARM AND INTAKE ########### */

    // X - pull cube, Y - pull cone
    // A - push cube, B - push cone
    if (takeCon.getXButton()) {
      intakeMotor.set(intakeSpeedPull);
    } else if (takeCon.getBButton()) {
      intakeMotor.set(intakeSpeedPush);
    } else if (takeCon.getAButton()) {
      intakeMotor.set(-intakeSpeedPush);
    } else if (takeCon.getYButton()) {
      intakeMotor.set(-intakeSpeedPull);
    } else {
      intakeMotor.set(0);
    }

    // (arm)
    pov = takeCon.getPOV();
    double LeftY = takeCon.getLeftY();

    if (Math.abs(LeftY) >= 0.1) {
      armMotor.set(LeftY * -armSpeed);
    } else if (pov == 180 || pov == 225 || pov == 135) {
      armMotor.set(-armSpeed);
    } else if (pov == 0 || pov == 315 || pov == 45) {
      armMotor.set(armSpeed);
    } else {
      armMotor.set(0);
    }

    armSpeed = SmartDashboard.getNumber("arm speed", armSpeed);
    if (takeCon.getLeftBumperPressed() && armSpeed > 0.2) {
      armSpeed -= 0.1;
      SmartDashboard.putNumber("arm speed", armSpeed);
    } else if (takeCon.getRightBumperPressed() && armSpeed < 1) {
      armSpeed += 0.1;
      SmartDashboard.putNumber("arm speed", armSpeed);
    }

    SmartDashboard.putNumber("arm motor temp", armMotor.getMotorTemperature());
    SmartDashboard.putNumber("intake motor temp", intakeMotor.getMotorTemperature());
    SmartDashboard.putNumber("intake speed push", intakeSpeedPush);
    SmartDashboard.putNumber("intake speed pull", intakeSpeedPull);

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
