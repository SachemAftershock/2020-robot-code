package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.StraightThenRotateAutoPath;
import frc.robot.commands.LinearDriveCommand;
import frc.robot.subsystems.DriveSubsystem;

public class AutoSelector {

    enum AutoPath {
        NOTHING, STRAIGHT, STRAIGHT_THEN_TURN
    }

    private AutoPath mSelectedAutoScenario, mPrevAutoScenario;

    private SendableChooser<AutoPath> mAutoChooser;

    public AutoSelector() {
        mPrevAutoScenario = null;

        mAutoChooser = new SendableChooser<>();
        mAutoChooser.setDefaultOption("No Path", AutoPath.NOTHING);
        mAutoChooser.addOption("Straight", AutoPath.STRAIGHT);
        mAutoChooser.addOption("Straight then Turn", AutoPath.STRAIGHT_THEN_TURN);

        SmartDashboard.putData("Auto Path", mAutoChooser);
    }

    public void selectAuto() {
        mSelectedAutoScenario = mAutoChooser.getSelected();
        if(mPrevAutoScenario != mSelectedAutoScenario) {
            System.out.println("Changing Auto Path: " + mSelectedAutoScenario.name());
        }
        mPrevAutoScenario = mSelectedAutoScenario;
    }

    public Command getSelectedAutoCommand() {
        switch(mSelectedAutoScenario) {
            case STRAIGHT:
                return new LinearDriveCommand(DriveSubsystem.getInstance(), 10);
            case STRAIGHT_THEN_TURN:
                return new StraightThenRotateAutoPath();
            case NOTHING:
            default:
                return null;
        }
    }
}