package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.ComplexAutoPath;
import frc.robot.auto.StraightThenRotateAutoPath;
import frc.robot.commands.drive.LinearDriveCommand;
import frc.robot.subsystems.DriveSubsystem;

public class AutoSelector {

    enum AutoPath {
        eNothing, eStraight, eStraightThenTurn, eComplexPath
    }

    private AutoPath mSelectedAutoScenario, mPrevAutoScenario;

    private SendableChooser<AutoPath> mAutoChooser;

    public AutoSelector() {
        mPrevAutoScenario = null;

        mAutoChooser = new SendableChooser<>();
        mAutoChooser.setDefaultOption("No Path", AutoPath.eNothing);
        mAutoChooser.addOption("Straight", AutoPath.eStraight);
        mAutoChooser.addOption("Straight then Turn", AutoPath.eStraightThenTurn);
        mAutoChooser.addOption("Complex Path", AutoPath.eComplexPath);

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
            case eStraight:
                return new LinearDriveCommand(DriveSubsystem.getInstance(), 10);
            case eStraightThenTurn:
                return new StraightThenRotateAutoPath();
            case eComplexPath:
                return new ComplexAutoPath();
            case eNothing:
            default:
                return null;
        }
    }
}