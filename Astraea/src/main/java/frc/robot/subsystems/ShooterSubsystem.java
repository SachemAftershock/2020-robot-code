package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    //private CANSparkMax mAzimuthMotor, mLaunchMotor;
    //private DoubleSolenoid mElevationSolenoid;

    public ShooterSubsystem() {
        //mAzimuthMotor = new CANSparkMax(Constants.kTurretMotor, MotorType.kBrushless);
        //mLaunchMotor = new CANSparkMax(Constants.kLauncherMotor, MotorType.kBrushless);

        //mElevationSolenoid = new DoubleSolenoid(Constants.kPcmId, Constants.kShooterElevationDoubleSolenoidForward, Constants.kShooterElevationDoubleSolenoidReverse);
        //addChild("Ball Shooter Aim Selector Double Solenoid",mElevationSolenoid);

    }

    @Override
    public void periodic() {

    }

    public void maintainTargetRPM() {
    }

    public void turnToAzimuth(double theta) {

    }

    public void toggleElevation() {
        /*if(mElevationSolenoid.get() == Value.kForward) {
            mElevationSolenoid.set(Value.kReverse);
        } else {
            mElevationSolenoid.set(Value.kForward);
        }*/
    }
}

