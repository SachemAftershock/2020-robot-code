package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SuperstructureConstants.IntakeConstants;
import frc.robot.commands.superstructure.intake.DeployIntakeCommand;
import frc.robot.commands.superstructure.intake.EjectIntakeCommand;
import frc.robot.commands.superstructure.intake.IngestIntakeCommand;
import frc.robot.commands.superstructure.intake.RetractIntakeCommand;
import frc.robot.commands.superstructure.intake.StopIntakeCommand;

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
        mIntakeExtender = new DoubleSolenoid(Constants.kPcmAId, IntakeConstants.kIntakeExtenderForwardId, IntakeConstants.kIntakeExtenderReverseId);
        addChild("Ball Floor Harvestor Deployment Double Solenoid",mIntakeExtender);

        mIntakeMotor = new WPI_TalonSRX(IntakeConstants.kIntakeMotorId);

        mIntakeExtender.set(Value.kReverse);
    }

    @Override
    public void init() {
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
        SmartDashboard.putBoolean("Is Intake Running", mIntakeMotor.get() != 0);
    }

    @Override
    public void runTest() {
        SmartDashboard.putData("Deploy Intake", new DeployIntakeCommand(getInstance()));
        SmartDashboard.putData("Retract Intake", new RetractIntakeCommand(getInstance()));
        SmartDashboard.putData("Run Intake", new IngestIntakeCommand(getInstance()));
        SmartDashboard.putData("Eject Intake", new EjectIntakeCommand(getInstance()));
        SmartDashboard.putData("Stop Intake", new StopIntakeCommand(getInstance()));
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

