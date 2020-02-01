package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Util;
import frc.robot.subsystems.TurretSubsystem;

public class DriveTurretCommand extends CommandBase {

    TurretSubsystem mTurret;
    XboxController mController;

    public DriveTurretCommand(TurretSubsystem turret, XboxController controller) {
        mTurret = turret;
        mController = controller;
        addRequirements(mTurret);
    }

    @Override
    public void execute() {
        mTurret.manualControl(Util.deadband(mController.getX(Hand.kLeft), Constants.kDeadbandTolerance));
    }

    @Override
    public void end(boolean interrupted) {
        mTurret.manualControl(0.0);
    }
}