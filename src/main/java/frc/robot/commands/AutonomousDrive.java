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
    private final PIDController drivePID = new PIDController(0.01, 0.00001, 0.0);

    private final double driveWheelCircumference = 47.4694649957;
    private final int startingPosition;

    private int stage = 1;
    private long startingTime = 0;
    private long currentTime = 0;
    private long tick;
    private double startingPitch;
    private double currentAngle;

    public static final int B1 = 1, B2 = 2, B3 = 3, R1 = 4, R2 = 5, R3 = 6;

    /**
     * The constructor of autonomous command.
     * Currently the parameters aren't subsystems,
     * but that's simply because we're too bad at coding.
     * I'll try to teach the juniors how to write command robot during off season
     * #TODO
     * 
     * @param drive  put the whole drive here as if we're the worst programmer ever,
     *               even though we actually are
     * @param arm    the neo motor that controls the arm
     * @param intake the neo motor that controls the intake, also this is like the
     *               laziest javadoc ever istg
     * @param pos    indicating the initial position of the drive, use `AutonomousDrive.B1`, `AutonomousDrive.R3` etc
     */
    public AutonomousDrive(DifferentialDrive drive, CANSparkMax arm, CANSparkMax intake, AHRS t, int pos) { 
        m_drive = drive;
        m_arm = arm;
        m_intake = intake;
        ahrs = t;
        startingPosition = pos;

        drivePID.setTolerance(10, 10);

        ahrs.reset();

        wheelSize = driveWheelCircumference;
    }

    @Override
    public void initialize() {
        startingTime = System.currentTimeMillis();
        currentTime = startingTime;
        startingPitch = ahrs.getPitch();
        currentAngle = ahrs.getAngle();
        stage = 1;
        tick = 0;
    }

    @Override
    public void execute() {
        if (System.currentTimeMillis() - startingTime > 15000) return;

        tick = System.currentTimeMillis() - currentTime;

        switch (startingPosition) {
        case B1:
            this.B1drive();
        break;
        case B2:
            this.B2drive();
        break;
        case B3:
            this.B3drive();
        break;
        case R1:
            this.R1drive();
        break;
        case R2:
            this.R2drive();
        break;
        case R3:
            this.R3drive();
        break;
        default:
        break;
        }
    }

    private void B1drive() {
        switch (stage) {
        case 1:
            m_arm.set(0.6);
            if (tick > 1200) {
                m_arm.set(0);
                this.nextStage();
            }
        break;
        case 2:
            m_drive.tankDrive(-0.7, -0.7);
            if (tick > 600) {
                m_drive.tankDrive(0, 0);
                this.nextStage();
            }
        break;
        case 3:
            if (tick > 100) this.nextStage();
        break;
        case 4:
            // cone +, cube -
            m_intake.set(-0.8);
            if (tick > 800) {
                m_intake.set(0);
                this.nextStage();
            }
        break;
        case 5:
            m_drive.tankDrive(0.8, 0.8);
            if (tick > 500) this.nextStage();
        case 6:
            m_arm.set(-0.5);
            m_drive.tankDrive(0.8, 0.8);
            if (tick > 1000) {
                m_arm.set(0);
                m_drive.tankDrive(0, 0);
                this.nextStage();
            }
        break;
        case 7:
            if (this.rotateDrive(-90)) this.nextStage();
        break;
        case 8:
            m_drive.tankDrive(1, 1);
            if (tick > 400) this.nextStage();
        break;
        case 9:
            if (this.rotateDrive(-90)) this.nextStage();
        break;
        case 10:
            m_drive.tankDrive(1, 1);
            if (ahrs.getPitch() - this.startingPitch > 11) {
                this.nextStage();
            }
        break;
        case 11:
            this.doBalance();
        break;
        }
    }

    private void B2drive() {
        System.out.println("autonomous drive for B2 position is still being worked in progress");
    }

    private void B3drive() {
        switch (stage) {
        case 1: // move forward
            m_drive.tankDrive(0.5, 0.5);
            if (tick > 2000) this.nextStage();
        break;
        case 2: // rotate right
            this.rotateDrive(-100);
            if (Math.abs(ahrs.getRawGyroZ()) < 30 && tick > 300) this.nextStage();
            System.out.println(ahrs.getRawGyroZ());
        break;
        case 3: // move forward
            m_drive.tankDrive(0.5,
             0.5);
            if (tick > 2000) this.nextStage();
        break;
        case 4: // rotate right
            this.rotateDrive(-90);
            if (Math.abs(ahrs.getRawGyroZ()) < 30 && tick > 300) this.nextStage();
            System.out.println(ahrs.getRawGyroZ());
        break;
        case 5: // move forward
            m_drive.tankDrive(0.5, 0.5);
            if (tick > 800) this.nextStage();
        break;
        default:
        break;
        }
    }

    private void R1drive() {
        switch (stage) {
        case 1: // move forward
            m_drive.tankDrive(1, 1);
            if (tick > 2000) this.nextStage();
        break;
        case 2: // rotate right
            m_drive.tankDrive(1, -1);
            if (this.getZAngle() > 90) this.nextStage();
        break;
        case 3: // move forward
            m_drive.tankDrive(1, 1);
            if (tick > 1000) this.nextStage();
        break;
        case 4: // rotate right
            m_drive.tankDrive(1, -1);
            if (this.getZAngle() > 90) this.nextStage();
        break;
        case 5: // try to get DOCKED
            m_drive.tankDrive(1, 1);
            if (tick > 2000) this.nextStage();
        break;
        case 6: // try to get ENGAGED
            this.doBalance();
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
                if (tick > 2000) this.nextStage();
                break;
            case 2: // rotate left
                m_drive.tankDrive(-1, 1);
                if (this.getZAngle() < 90) this.nextStage();
                break;
            case 3: // move forward
                m_drive.tankDrive(1, 1);
                if (tick > 1000) this.nextStage();
                break;
            case 4: // rotate left
                m_drive.tankDrive(-1, 1);
                if (this.getZAngle() < 90) this.nextStage();
                break;
            case 5: // try to get DOCKED
                m_drive.tankDrive(1, 1);
                if (tick > 2000) this.nextStage();
                break;
            case 6: // try to get ENGAGED
                this.doBalance();
                break;
            default:
                break;
        }
    }
    
    public void nextStage() {
        currentTime = System.currentTimeMillis();
        currentAngle = ahrs.getAngle();
        drivePID.reset();
        stage++;
    }

    private double getZAngle() {
        return ahrs.getAngle() - currentAngle;
    }

    public void doBalance() {
        double speed = drivePID.calculate(ahrs.getPitch(), startingPitch);
        m_drive.tankDrive(speed, speed);
    }

    /**
     * 
     * @param degree the degree you want the drive to turn
     * @return       return whether or not the drive is at the correct degree
     */
    public boolean rotateDrive(int degree) {
        double speed = drivePID.calculate(this.getZAngle(), degree);
        m_drive.tankDrive(-speed, speed);
        return Math.abs(ahrs.getRawGyroZ()) < ((degree > 0)? 30: -30) && tick > 300;
    }
}
