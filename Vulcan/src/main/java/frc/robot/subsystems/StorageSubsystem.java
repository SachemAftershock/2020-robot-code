package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class StorageSubsystem extends SubsystemBase {

    private final VictorSPX mFeeder;

    public StorageSubsystem() {
        mFeeder = new WPI_VictorSPX(Constants.kFeederMotorId);
    }
}