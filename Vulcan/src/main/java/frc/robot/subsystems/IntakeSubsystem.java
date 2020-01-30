package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class IntakeSubsystem extends SubsystemBase {

    private static IntakeSubsystem mInstance;

    private DoubleSolenoid mIntakeExtender;
    private WPI_VictorSPX mIntakeMotor;

    private final double kIntakeSpeed = 0.5;

    public IntakeSubsystem() {
        mIntakeExtender = new DoubleSolenoid(Constants.kPcmId, Constants.kIntakeExtenderForwardId, Constants.kIntakeExtenderReverseId);
        addChild("Ball Floor Harvestor Deployment Double Solenoid",mIntakeExtender);

        mIntakeMotor = new WPI_VictorSPX(Constants.kIntakeMotorId);

        mIntakeExtender.set(Value.kReverse);
    }

    @Override
    public void periodic() {

    }

    public void startIntakeMotor() {
        mIntakeMotor.set(ControlMode.PercentOutput, kIntakeSpeed);
    }

    public void ejectIntakeMotor() {
        mIntakeMotor.set(ControlMode.PercentOutput, -kIntakeSpeed);
    }

    public void stopIntakeMotor() {
        mIntakeMotor.set(ControlMode.PercentOutput, 0.0);
    }

    public void toggleIntakeExtender() {
        if(mIntakeExtender.get() == Value.kForward) {
            mIntakeExtender.set(Value.kReverse);
        } else {
            mIntakeExtender.set(Value.kForward);
        }
    }

    public static IntakeSubsystem getInstance() {
        if(mInstance == null) {
            mInstance = new IntakeSubsystem();
        }
        return mInstance;
    }
}

