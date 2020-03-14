package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticConstants;
import frc.robot.Constants.SuperstructureConstants.IntakeConstants;


/**
 * Intake Subsystem
 * 
 * @author Shreyas Prasad
 * 
 * @see SuperstructureSubsystem
 * 
 * @see StorageSubsystem
 */
public class IntakeSubsystem extends SubsystemBase implements SubsystemInterface {

    private static IntakeSubsystem mInstance;

    private DoubleSolenoid mIntakeExtender;
    private WPI_TalonSRX mIntakeMotor;

    /**
     * Constructor for the IntakeSubsystem Class
     */
    private IntakeSubsystem() {
        mIntakeExtender = new DoubleSolenoid(PneumaticConstants.kPcmId, IntakeConstants.kIntakeForwardId, IntakeConstants.kIntakeReverseId);
        addChild("Ball Floor Harvestor Deployment Double Solenoid",mIntakeExtender);

        mIntakeMotor = new WPI_TalonSRX(IntakeConstants.kIntakeMotorId);
        mIntakeMotor.setInverted(InvertType.InvertMotorOutput);

        mIntakeExtender.set(Value.kReverse);
    }

    @Override
    public void init() {
        mIntakeMotor.set(ControlMode.PercentOutput, 0.0);
        mIntakeExtender.set(Value.kReverse);
    }

    /**
     * Runs Intake at a constant speed
     */
    public void runIntakeMotor() {
        mIntakeMotor.set(ControlMode.PercentOutput, IntakeConstants.kIntakeSpeed);
    }

    /**
     * Runs Intake in the opposite direction at a constant speed
     */
    public void ejectIntakeMotor() {
        mIntakeMotor.set(ControlMode.PercentOutput, -IntakeConstants.kIntakeSpeed);
    }

    /**
     * Stops the Intake from Running
     */
    public void stopIntakeMotor() {
        mIntakeMotor.set(ControlMode.PercentOutput, 0.0);
    }

    /**
     * Deploys the intake & starts the Intake Motor
     */
    public void deployIntake() {
        mIntakeExtender.set(Value.kForward);
        runIntakeMotor();
    }

    /**
     * Retracts the Intake back inside the Frame Perimeter & stops the Intake Motor
     */
    public void retractIntake() {
        mIntakeExtender.set(Value.kReverse);
        stopIntakeMotor();
    }

    /**
     * Gets whether the Intake is deployed or not
     * 
     * @return <i> true </i> if the Intake is deployed; <i> false </i> otherwise
     */
    public boolean isIntakeDeployed() {
        return mIntakeExtender.get() == Value.kForward;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putData(getInstance());
        SmartDashboard.putBoolean("Is Intake Deployed", isIntakeDeployed());
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    /**
     * @return IntakeSubsystem Singleton Instance
     */
    public synchronized static IntakeSubsystem getInstance() {
        if(mInstance == null) {
            mInstance = new IntakeSubsystem();
        }
        return mInstance;
    }
}

