// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
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

  private XboxController con1 = new XboxController(0);
  private XboxController con2 = new XboxController(1);

  private double deepTrigger =0.8;//Trigger 觸發最小值

  private static final int m_intake = 1; //手掌馬達編號
  private static final int m_arm = 2;    //手臂馬達編號
  
  private static final int m_rightFront = 1;//右前
  private static final int m_leftFront = 11;//左前
  private static final int m_rightRear = 9; //右後
  private static final int m_leftRear = 7;  //左後

  private CANSparkMax intakeMotor;
  private CANSparkMax armMotor;

  private double intakeSpeed = 0.5; //手掌初始速度
  private double armSpeed = 0.5; //手臂初始速度
  private double drivespeed = 0.5; //底盤初始速度

  private int pov;

  private WPI_VictorSPX motorRightRear;
  private WPI_VictorSPX motorRightFront;
  private WPI_VictorSPX motorLeftFront;
  private WPI_VictorSPX motorLeftRear;

  private MotorControllerGroup leftGroup;
  private MotorControllerGroup rightGroup;
  private DifferentialDrive drive;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    intakeMotor = new CANSparkMax(m_intake, MotorType.kBrushless);
    armMotor = new CANSparkMax(m_arm, MotorType.kBrushless); 

    motorRightRear = new WPI_VictorSPX(m_rightRear);
    motorRightFront = new WPI_VictorSPX(m_rightFront);
    motorLeftFront = new WPI_VictorSPX(m_leftFront);
    motorLeftRear = new WPI_VictorSPX(m_leftRear);

    leftGroup = new MotorControllerGroup(motorLeftFront, motorLeftRear);
    rightGroup = new MotorControllerGroup(motorRightFront, motorRightRear);
    drive = new DifferentialDrive(leftGroup, rightGroup);

    intakeMotor.restoreFactoryDefaults();
    armMotor.restoreFactoryDefaults();

    leftGroup.setInverted(true); //左馬達組啟用反轉
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
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if(con1.getLeftBumperPressed() && drivespeed > 0.1) {//
      drivespeed -= 0.1;
    }
    if(con1.getRightBumperPressed() && drivespeed < 1) {
      drivespeed += 0.1;
    }

   
    /*if(con1.getLeftTriggerAxis() >= deepTrigger){
      drive.tankDrive(con1.getLeftTriggerAxis() * drivespeed, con1.getLeftTriggerAxis() * -drivespeed);
    }
    else if(con1.getRightTriggerAxis() >= deepTrigger){
      drive.tankDrive(con1.getRightTriggerAxis() * -drivespeed, con1.getRightTriggerAxis()*drivespeed);
    }

    if (Math.abs(con1.getLeftY()) > 0.1 || Math.abs(con1.getRightY()) > 0.1) {
      drive.tankDrive(con1.getLeftY()*drivespeed, con1.getRightY()*drivespeed);
    } else if(con1.getLeftTriggerAxis()<deepTrigger||con1.getRightTriggerAxis()<deepTrigger){
      drive.tankDrive(0, 0);
    }*/
    if (con1.getLeftTriggerAxis() >0.1) // Handle robot rotation
    {
      drive.tankDrive(drivespeed, -drivespeed);
    }
    if (con1.getRightTriggerAxis() >0.1)
    {
      drive.tankDrive(-drivespeed, drivespeed);
    }
    if (con1.getLeftTriggerAxis()<=0.1 && con1.getRightTriggerAxis()<=0.1) // If not rotating, drive
    {   
      if (Math.abs(con1.getLeftY())>0.1 || Math.abs(con1.getRightY())>0.1)
      {
        drive.tankDrive(con1.getLeftY()*drivespeed, con1.getRightY()*drivespeed);
      }
      else 
      {
        drive.tankDrive(0, 0);
      }
    }
    
    if (con1.getLeftTriggerAxis()>0.1 && con1.getRightTriggerAxis()>0.1)
    {
      drive.tankDrive(0, 0);
    }

 
    if(con2.getXButton() || con2.getBButton()) {
      intakeMotor.set(-intakeSpeed);
    } else if(con2.getAButton() || con2.getYButton()) {
      intakeMotor.set(intakeSpeed);
    } else {
      intakeMotor.set(0);
    }

    if((con2.getYButton() && con2.getBButton()) || con2.getXButton() && con2.getAButton()){
      intakeMotor.set(0);
    }

    if(con2.getLeftBumperPressed() && armSpeed > 0.1){
      armSpeed -= 0.1;
    }
    if(con2.getRightBumperPressed() && armSpeed < 1) {
      armSpeed += 0.1;
    }

    pov = con2.getPOV();
    if(Math.abs(con2.getLeftY()) > 0.1) {
      armMotor.set(con2.getLeftY() *armSpeed);
    } else if(pov == 180 || pov == 225 || pov == 135) {
      armMotor.set(armSpeed);
    } else if(pov == 0 || pov == 315 || pov == 45) {
      armMotor.set(-armSpeed);
    } else {
      armMotor.set(0);
    }

    SmartDashboard.putNumber("drive", drivespeed);
    SmartDashboard.putNumber("intake", intakeSpeed);
    SmartDashboard.putNumber("arm", armSpeed);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

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
