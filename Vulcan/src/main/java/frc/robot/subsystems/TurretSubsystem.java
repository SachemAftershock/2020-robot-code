package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {

    private static TurretSubsystem mInstance;

    private WPI_VictorSPX mTurret;
    private DutyCycleEncoder mEncoder;
    private PIDController mPid;
    
    private final double[] mGains = {0.0, 0.0, 0.0};
    private final double kTurretEpsilon = 4.0; //TODO: Find the right value
    private final double kTurretDegreesPerEncoderRotation = 44.0; //TODO: Find the right value

    public TurretSubsystem() {
        mTurret = new WPI_VictorSPX(Constants.kTurretMotorId);
        mTurret.setNeutralMode(NeutralMode.Brake);

        mEncoder = new DutyCycleEncoder(new DigitalInput(Constants.kTurretEncoderDioId));
        mEncoder.setDistancePerRotation(kTurretDegreesPerEncoderRotation);


        mPid = new PIDController(mGains[0], mGains[1], mGains[2]);
        mPid.setTolerance(kTurretEpsilon);
    }

    @Override
    public void periodic() {
    }

    public void manualControl(double pow) {
        mTurret.set(ControlMode.PercentOutput, pow);
    }

    public void turnToDegree(double theta) {
        
    }

    public static TurretSubsystem getInstance() {
        if(mInstance == null) {
            mInstance = new TurretSubsystem();
        }
        return mInstance;
    }

}