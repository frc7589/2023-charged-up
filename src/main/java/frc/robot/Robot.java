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

import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;

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

  /*
  `m_nameOfSomething` names are not for constants, use `A_NAME_FOR_CONSTANT` instead 
  Therefore, I suggest putting these constants into a different file and name it `Constant.java`,
  so you don't have to make everything CAPS. Also it will make the code look less messy.
  You should have learnt how to do that in class, so you might want to do it as quickly as possible.
  */
  private static final int m_intake = 1; // 手掌馬達&編號
  private static final int m_arm = 2; // 手臂馬達&編號

  private static final int m_rightFront = 7;
  private static final int m_leftFront = 9;
  private static final int m_rightRear = 8;
  private static final int m_leftRear = 1;

  private double intakeSpeed = 0.6; // intake initial speed
  private double armSpeed = 0.2; // arm initial speed
  private double drivespeed = 0.4; // car initial speed

  private int pov;

  private XboxController driveCon;
  private XboxController takeCon;

  private CANSparkMax intakeMotor;
  private CANSparkMax armMotor;

  private WPI_VictorSPX motorRightRear;
  private WPI_VictorSPX motorRightFront;
  private WPI_VictorSPX motorLeftFront;
  private WPI_VictorSPX motorLeftRear;

  private MotorControllerGroup leftGroup;
  private MotorControllerGroup rightGroup;
  private DifferentialDrive drive;

  private AHRS ahrs;

  private AutonomousDrive autoCommands;

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

    intakeMotor = new CANSparkMax(m_intake, MotorType.kBrushless);
    armMotor = new CANSparkMax(m_arm, MotorType.kBrushless);

    motorRightRear = new WPI_VictorSPX(m_rightRear);
    motorRightFront = new WPI_VictorSPX(m_rightFront);
    motorLeftFront = new WPI_VictorSPX(m_leftFront);
    motorLeftRear = new WPI_VictorSPX(m_leftRear);

    leftGroup = new MotorControllerGroup(motorLeftFront, motorLeftRear);
    rightGroup = new MotorControllerGroup(motorRightFront, motorRightRear);
    drive = new DifferentialDrive(leftGroup, rightGroup);

    leftGroup.setInverted(true); // 左馬達組啟用反轉

    ahrs = new AHRS(SPI.Port.kMXP);

    autoCommands = new AutonomousDrive(drive, armMotor, intakeMotor, ahrs, AutonomousDrive.B1);
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
  public void robotPeriodic() {
  }

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

    autoCommands.initialize();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    autoCommands.execute();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    /* ########## CAR ########## */

    if (driveCon.getLeftBumperPressed() && drivespeed >= 0.3) {
      drivespeed -= 0.1;
    }
    if (driveCon.getRightBumperPressed() && drivespeed < 1) {
      drivespeed += 0.1;
    }

    if (driveCon.getLeftTriggerAxis() >= 0.1 && driveCon.getRightTriggerAxis() >= 0.1) {
      drive.tankDrive(0, 0);
    }

    if (driveCon.getLeftTriggerAxis() >= 0.1) {
      drive.tankDrive(driveCon.getLeftTriggerAxis() * drivespeed, driveCon.getRightTriggerAxis() * -drivespeed);
    } else if (driveCon.getRightTriggerAxis() >= 0.1) {
      drive.tankDrive(-drivespeed, drivespeed);
    }

    if (driveCon.getLeftTriggerAxis() < 0.1 && driveCon.getRightTriggerAxis() < 0.1) {
      if (Math.abs(driveCon.getLeftY()) >= 0.1 || Math.abs(driveCon.getRightY()) >= 0.1) {
        drive.tankDrive(driveCon.getLeftY() * drivespeed, driveCon.getRightY() * drivespeed);
      } else {
        drive.tankDrive(0, 0);
      }
    }

    /* ############# ARM AND INTAKE ########### */

    if (takeCon.getXButton() || takeCon.getBButton()) {
      intakeMotor.set(-intakeSpeed);
    } else if (takeCon.getAButton() || takeCon.getYButton()) {
      intakeMotor.set(intakeSpeed);
    } else {
      intakeMotor.set(0);
    }

    // A && B || X && Y (intake)
    if ((takeCon.getYButton() && takeCon.getBButton()) || takeCon.getXButton() && takeCon.getAButton()) {
      intakeMotor.set(0);
    }

    if (takeCon.getLeftBumperPressed() && armSpeed >= 0.3) {
      armSpeed -= 0.1;
    }
    if (takeCon.getRightBumperPressed() && armSpeed < 1) {
      armSpeed += 0.1;
    }
    
    // (arm)
    pov = takeCon.getPOV();
    if (Math.abs(takeCon.getLeftY()) > 0.1) {
      armMotor.set(takeCon.getLeftY() * -armSpeed);
    } else if (pov == 180 || pov == 225 || pov == 135) {
      armMotor.set(-armSpeed);
    } else if (pov == 0 || pov == 315 || pov == 45) {
      armMotor.set(armSpeed);
    } else {
      armMotor.set(0);
    }
    SmartDashboard.putNumber("drive speed", drivespeed);
    SmartDashboard.putNumber("intake speed", intakeSpeed);
    SmartDashboard.putNumber("arm speed", armSpeed);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
