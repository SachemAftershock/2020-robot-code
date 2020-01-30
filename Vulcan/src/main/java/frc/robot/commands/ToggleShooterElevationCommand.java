package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ControllerRumble;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.XboxController;

public class ToggleShooterElevationCommand extends CommandBase {

    private ShooterSubsystem mShooter;
    private XboxController mController;
    private boolean mIsFinished;

    public ToggleShooterElevationCommand(ShooterSubsystem shooter, XboxController controller) {
        mShooter = shooter;
        mController = controller;
        addRequirements(mShooter);
        mIsFinished = false;
    }

    @Override
    public void execute() {
        mShooter.toggleElevation();
        (new ControllerRumble(mController, 1)).start();;
        mIsFinished = true;
    }

    @Override
    public boolean isFinished() {
        return mIsFinished;
    }
}
