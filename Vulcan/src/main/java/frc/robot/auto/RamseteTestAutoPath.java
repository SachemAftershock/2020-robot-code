package frc.robot.auto;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class RamseteTestAutoPath extends CommandBase {
    private RamseteCommand mRamseteCommand;
    private final DriveSubsystem mDrive;
    public RamseteTestAutoPath(DriveSubsystem drive) {
        mDrive = drive;
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                       DriveConstants.kvVoltSecondsPerMeter,
                                       DriveConstants.kaVoltSecondsSquaredPerMeter), 
            mDrive.getKinematics(),
            DriveConstants.kMaxVoltage);

        TrajectoryConfig config = new TrajectoryConfig(DriveConstants.kMaxSpeed, DriveConstants.kMaxAcceleration)
                                    .setKinematics(mDrive.getKinematics())
                                    .addConstraint(autoVoltageConstraint);
        
        Trajectory testTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0,0, new Rotation2d(0)),
        List.of(
            new Translation2d(1, 1),
            new Translation2d(2, -1)
        ),
        new Pose2d(3, 0, new Rotation2d(0)),
        config
        );

        mRamseteCommand = new RamseteCommand(
            testTrajectory,
            mDrive::getPoseMeters, 
            new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
            new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter),
            mDrive.getKinematics(),
            mDrive::getWheelSpeedsMetersPerSecond,
            new PIDController(DriveConstants.kPDriveVel, 0, 0), 
            new PIDController(DriveConstants.kPDriveVel, 0, 0), 
            mDrive::tankDriveVolts,
            mDrive
        );
    }

    public Command getCommand() {
        return mRamseteCommand.andThen(() -> mDrive.tankDriveVolts(0, 0));
    }
}