package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

public final class Constants {
    /*Throughout all code: All units are assumed to be in the following units:
        - Distance: Meters
        - Angle: Degrees
        - Time: Seconds
        - Speed: Meters/Second
        - Acceleration: Meters/Second^2
        - Angular Speed: Radians/Second
    
        Unless otherwise specified
    */

    // XboxController Constants
    public static final int kControllerPrimaryId = 0;
    public static final int kControllerSecondaryId = 1;

    public static final double kDeadbandTolerance = 0.05;

    //Pneumatic Constants
    public static final int kPcmAId = 0;
    public static final int kPcmBId = 1;

    public static final class CollisionAvoidanceConstants {
        public static final int kCollisionUltrasonicId = 1;
        public static final double kUltrasonicValueToInches = 0.125;
    }

    public static final class DriveConstants {
        public static final int kDriveMotorPortAId = 1;
        public static final int kDriveMotorPortBId = 2;
        public static final int kDriveMotorPortCId = 3;
        public static final int kDriveMotorStarboardAId = 4;
        public static final int kDriveMotorStarboardBId = 5;
        public static final int kDriveMotorStarboardCId = 6;

        public static final double kWheelDiameter = Units.inchesToMeters(6.0);
        public static final double kWheelCircumference = kWheelDiameter * Math.PI;
        public static final double kDriveEncoderPPR = 42.0;
        public static final double kWheelRadius = Units.inchesToMeters(3.0);
        public static final double kTrackWidth = Units.inchesToMeters(19.125); //Distance in between wheels
        public static final double kLowGearRatio = 1.0;
        public static final double kHighGearRatio = 1.0; //TODO: If switched to Rev Through Bore Encoder, do not need this
        
        public static final double kDriveEpsilon = 0.1; 
        public static final double kRotateEpsilon = 3.0;
        public static final double[] kLinearGains = {0.0, 0.0, 0.0};
        public static final double[] kRotationalGains = {0.0, 0.0, 0.0};
        public static final double kRampRateToMaxSpeed = 1.0;

        public static final double kRegularMaxSpeed = 1.0;
        public static final double kPrecisionMaxSpeed = 0.5;

        //Robot Characterization Variables
        public static final double ksVolts = 1.0;
        public static final double kvVoltSecondsPerMeter = 1.0;
        public static final double kaVoltSecondsSquaredPerMeter = 1.0;
        public static final double kPDriveVel = 1.0;//Tuned from above values
        public static final double kMaxSpeed = 1.0;
        public static final double kMaxAcceleration = 1.0;
        public static final double kRamseteB = 1.0;
        public static final double kRamseteZeta = 1.0;

        public static final double kMaxVoltage = 10;

        // Pneumatics, PCM A
        public static final int kGearShiftForwardId = 1;
        public static final int kGearShiftReverseId = 2;
    }

    public static final class SuperstructureConstants {
        public static final class ShooterConstants {
            public static final int kLauncherMotorId = 7;
            public static final int kFeederMotorId = 8;

            public static final int kLidarId = 0;

            public static final int kPidId = 0;
        }  

        public static final class TurretConstants {
            public static final int kTurretMotorId = 9;

            public static final int kTurretEncoderDioId = 0;
        }

        public static final class StorageConstants { 
            public static final int kBeltDriverMotorId = 10;

            public static final int kChamberDetectorId = 0;
            public static final int kPreChamberDetectorId = 1;
            public static final int kIntakeDetectorId = 2;
            public static final int kEntryDetectorId = 3;

            //Pneumatics PCM A
            public static final int kBallValveAForwardId = 3;
            public static final int kBallValveAReverseId = 4;
        
            public static final int kBallValveBForwardId = 5;
            public static final int kBallValveBReverseId = 6;
        }

        public static final class IntakeConstants {
            public static final int kIntakeMotorId = 11;

            //Pneumatics PCM A   
            public static final int kIntakeExtenderForwardId = 7;
            public static final int kIntakeExtenderReverseId = 8;
        }
    }

    public static final class WheelControllerConstants {
        public static final int kWheelControllerId = 12;
        public static final int kUltrasonicId = 0;
        
        //Pneumatics PCM B
        public static final int kExtenderForwardId = 0;
        public static final int kExtenderReverseId = 1;
    }
}