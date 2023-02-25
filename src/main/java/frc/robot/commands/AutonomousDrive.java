package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.revrobotics.CANSparkMax;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;

public class AutonomousDrive extends CommandBase {
    private final DifferentialDrive m_drive;
    private final CANSparkMax m_arm, m_intake;
    private final AHRS ahrs;
    private final double wheelSize;
    private final PIDController drivePID = new PIDController(0.1, 0.00001, 1);

    private final double driveWheelCircumference = 47.4694649957;
    private final int startingPosition;

    private int stage = 1;
    private long startingTime = 0;
    private long tick;

    public static final int B1 = 1, B2 = 2, B3 = 3, R1 = 4, R2 = 5, R3 = 6;

    /**
     * The constructor of autonomous command.
     * Currently the parameters aren't subsystems,
     * but that's simply because we're too bad at coding.
     * I'll try to teach the juniors how to write command robot during off season #TODO
     * 
     * @param drive  put the whole drive here as if we're the worst programmer ever, even though we actually are
     * @param arm    the neo motor that controls the arm
     * @param intake the neo motor that controls the intake, also this is like the laziest javadoc ever istg
     * @param pos    indicating the initial position of the drive, use `AutonomousDrive.B1`, `AutonomousDrive.R3` etc
     */
    public AutonomousDrive(DifferentialDrive drive, CANSparkMax arm, CANSparkMax intake, AHRS t, int pos) {
        m_drive = drive;
        m_arm = arm;
        m_intake = intake;
        ahrs = t;
        startingPosition = pos; 

        ahrs.reset();

        wheelSize = driveWheelCircumference;

    }

    @Override
    public void initialize() {
        startingTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        tick = System.currentTimeMillis() - startingTime;

        if (tick > 15000) return;

        switch (startingPosition) {
        case B1:
            B1drive();
        break;
        case B2:
            B2drive();
        break;
        case B3:
            B3drive();
        break;
        case R1:
            R1drive();
        break;
        case R2:
            R2drive();
        break;
        case R3:
            R3drive();
        break;
        default:
        break;
        }
    }

    private void B1drive() {
        switch (stage) {
        case 1: // move forward
            m_drive.tankDrive(1, 1);
            if (tick > 2000) stage++;
        break;
        case 2: // rotate left
            m_drive.tankDrive(-1, 1);
            if (tick > 2500) stage++;
        break;
        case 3: // move forward
            m_drive.tankDrive(1, 1);
            if (tick > 3500) stage++;
        break;
        case 4: // rotate left
            m_drive.tankDrive(-1, 1);
            if (tick > 4000) stage++;
        break;
        case 5: // try to get DOCKED
            m_drive.tankDrive(1, 1);
            if (tick > 6000) stage++;
        break;
        case 6: // try to get ENGAGED
            double speed = drivePID.calculate(ahrs.getAngle(), 0);
            m_drive.tankDrive(speed, speed);
        break;
        default:
        break;
        }
    }

    private void B2drive() {
        System.out.println("autonomous drive for B2 position is still being worked in progress");
    }

    private void B3drive() {
        switch (stage) {
        case 1: // move forward
            m_drive.tankDrive(1, 1);
            if (tick > 2000) stage++;
        break;
        case 2: // rotate right
            m_drive.tankDrive(1, -1);
            if (tick > 2500) stage++;
        break;
        case 3: // move forward
            m_drive.tankDrive(1, 1);
            if (tick > 3500) stage++;
        break;
        case 4: // rotate right
            m_drive.tankDrive(1, -1);
            if (tick > 4000) stage++;
        break;
        case 5: // try to get DOCKED
            m_drive.tankDrive(1, 1);
            if (tick > 6000) stage++;
        break;
        case 6: // try to get ENGAGED 
            double speed = drivePID.calculate(ahrs.getAngle(), 0);
            m_drive.tankDrive(speed, speed);
        break;
        default:
        break;
        }
    }

    private void R1drive() {
        switch (stage) {
        case 1: // move forward
            m_drive.tankDrive(1, 1);
            if (tick > 2000) stage++;
        break;
        case 2: // rotate right
            m_drive.tankDrive(1, -1);
            if (tick > 2500) stage++;
        break;
        case 3: // move forward
            m_drive.tankDrive(1, 1);
            if (tick > 3500) stage++;
        break;
        case 4: // rotate right
            m_drive.tankDrive(1, -1);
            if (tick > 4000) stage++;
        break;
        case 5: // try to get DOCKED
            m_drive.tankDrive(1, 1);
            if (tick > 6000) stage++;
        break;
        case 6: // try to get ENGAGED 
            double speed = drivePID.calculate(ahrs.getAngle(), 0);
            m_drive.tankDrive(speed, speed);
        break;
        default:
        break;
        }
    }

    private void R2drive() {
        System.out.println("autonomous drive for R2 position is still being worked in progress");
    }

    private void R3drive() {
        switch (stage) {
        case 1: // move forward
            m_drive.tankDrive(1, 1);
            if (tick > 2000) stage++;
        break;
        case 2: // rotate left
            m_drive.tankDrive(-1, 1);
            if (tick > 2500) stage++;
        break;
        case 3: // move forward
            m_drive.tankDrive(1, 1);
            if (tick > 3500) stage++;
        break;
        case 4: // rotate left
            m_drive.tankDrive(-1, 1);
            if (tick > 4000) stage++;
        break;
        case 5: // try to get DOCKED
            m_drive.tankDrive(1, 1);
            if (tick > 6000) stage++;
        break;
        case 6: // try to get ENGAGED
            double speed = drivePID.calculate(ahrs.getAngle(), 0);
            m_drive.tankDrive(speed, speed);
        break;
        default:
        break;
        }
    }
}

