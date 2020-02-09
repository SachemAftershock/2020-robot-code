package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.SuperstructureConstants.IntakeConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * Intake Subsystem
 * @author Shreyas Prasad
 */
public class IntakeSubsystem extends SubsystemBase implements SubsystemInterface {

    private static IntakeSubsystem mInstance;

    private DoubleSolenoid mIntakeExtender;
    private WPI_VictorSPX mIntakeMotor;

    private IntakeSubsystem() {
        mIntakeExtender = new DoubleSolenoid(Constants.kPcmAId, IntakeConstants.kIntakeExtenderForwardId, IntakeConstants.kIntakeExtenderReverseId);
        addChild("Ball Floor Harvestor Deployment Double Solenoid",mIntakeExtender);

        mIntakeMotor = new WPI_VictorSPX(IntakeConstants.kIntakeMotorId);

        mIntakeExtender.set(Value.kReverse);
    }

    @Override
    public void init() {
    }

    public void runIntakeMotor() {
        mIntakeMotor.set(ControlMode.PercentOutput, IntakeConstants.kIntakeSpeed);
    }

    public void ejectIntakeMotor() {
        mIntakeMotor.set(ControlMode.PercentOutput, -IntakeConstants.kIntakeSpeed);
    }

    public void stopIntakeMotor() {
        mIntakeMotor.set(ControlMode.PercentOutput, 0.0);
    }

    public void deployIntake() {
        mIntakeExtender.set(Value.kForward);
    }

    public void retractIntake() {
        mIntakeExtender.set(Value.kReverse);
    }

    public boolean isIntakeDeployed() {
        return mIntakeExtender.get() == Value.kForward;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Is Intake Deployed", isIntakeDeployed());
        SmartDashboard.putBoolean("Is Intake Running", mIntakeMotor.get() != 0);
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

