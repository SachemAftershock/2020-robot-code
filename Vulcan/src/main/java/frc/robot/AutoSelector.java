package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.ComplexAutoPath;
import frc.robot.auto.RamseteTestAutoPath;
import frc.robot.auto.StraightThenRotateAutoPath;
import frc.robot.commands.drive.LinearDriveCommand;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Class to select desired Autonomous Configuration at Startup
 * 
 * @author Shreyas Prasad
 */
public class AutoSelector {

    /**
     * Autonomous Paths that map to a Command Sequence
     * 
     * @author Shreyas Prasad
     */
    enum AutoPath {
        eNothing, eStraight, eStraightThenTurn, eComplexPath, eRamseteTest
    }

    private AutoPath mSelectedAutoScenario, mPrevAutoScenario;

    private SendableChooser<AutoPath> mAutoChooser;

    /**
     * Constructor for AutoSelector Class
     */
    public AutoSelector() {
        mPrevAutoScenario = null;

        mAutoChooser = new SendableChooser<>();
        mAutoChooser.setDefaultOption("No Path", AutoPath.eNothing);
        mAutoChooser.addOption("Straight", AutoPath.eStraight);
        mAutoChooser.addOption("Straight then Turn", AutoPath.eStraightThenTurn);
        mAutoChooser.addOption("Complex Path", AutoPath.eComplexPath);

        SmartDashboard.putData("Auto Path", mAutoChooser);
    }

    /**
     * Allows Operators to select one of several designed Autonomous Routines via SmartDashboard/Shuffleboard
     */
    public void selectAuto() {
        mSelectedAutoScenario = mAutoChooser.getSelected();
        if(mPrevAutoScenario != mSelectedAutoScenario) {
            System.out.println("Changing Auto Path: " + mSelectedAutoScenario.name());
        }
        mPrevAutoScenario = mSelectedAutoScenario;
    }

    /**
     * Decodes AutoPath Enum to a Command Sequence
     * 
     * @return Command Sequence for Autonomous
     */
    public Command getSelectedAutoCommand() {
        switch(mSelectedAutoScenario) {
            case eStraight:
                return new LinearDriveCommand(DriveSubsystem.getInstance(), 10);
            case eStraightThenTurn:
                return new StraightThenRotateAutoPath();
            case eComplexPath:
                return new ComplexAutoPath();
            case eRamseteTest:
                return (new RamseteTestAutoPath(DriveSubsystem.getInstance())).getCommand();
            case eNothing:
            default:
                return null;
        }
    }
}