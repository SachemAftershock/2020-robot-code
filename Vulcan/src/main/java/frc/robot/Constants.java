package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

public final class Constants {
    /*
    Throughout all code: All units are assumed to be in the following units:
        - Distance: Meters
        - Angle: Degrees
        - Time: Seconds
        - Speed: Meters/Second
        - Acceleration: Meters/Second^2
        - Angular Speed: Radians/Second
    
    Unless otherwise specified
    */

    public static final class ControllerConstants {
        public static final int kControllerPrimaryId = 0;
        public static final int kControllerSecondaryId = 1;

        public static final double kJoystickDeadbandTolerance = 0.15;
        public static final double kTriggerDeadbandTolerance = 0.25;
    }
    
    public static final class PneumaticConstants {
        public static final int kPcmId = 0;
    }

    //Ardunio I2C Address
    public static final class LEDConstants {
        public final static int kArduinoI2CAddress = 10;

        public static final double kTempModeRunTime = 3.0;
    }

    public static final class LimelightConstants {
        public static final String kShooterTableName = "limelight-shooter";
        public static final String kIntakeTableName = "limelight-intake";
    }

    public static final class CollisionAvoidanceConstants {
        public static final int kCollisionUltrasonicId = 3;
        public static final double kUltrasonicValueToInches = 0.125;
        public static final int kNumberOfDistanceSamples = 10;
        //TODO: Find good values for the below
        public static final double kCollisionStandoffSlowdownInches = 18.0;
        public static final double kColorWheelStandoffSlowdownInches = 12.0;
    }

    public static final class DriveConstants {
        public static final int kDriveMotorPortAId = 1;
        public static final int kDriveMotorPortBId = 2;
        public static final int kDriveMotorPortCId = 3;
        public static final int kDriveMotorStarboardAId = 4;
        public static final int kDriveMotorStarboardBId = 5;
        public static final int kDriveMotorStarboardCId = 6;

        public static final double kWheelRadiusInches = 3.0;
        public static final double kWheelDiameterInches = 6.0;
        public static final double kWheelCircumferenceInches = kWheelDiameterInches * Math.PI;
        public static final double kWheelDiameter = Units.inchesToMeters(kWheelDiameterInches);
        public static final double kWheelCircumference = kWheelDiameter * Math.PI;
        public static final double kWheelRadius = Units.inchesToMeters(3.0);
        public static final double kTrackWidth = Units.inchesToMeters(19.125); //Distance in between wheels
        public static final double kLowGearRatio = 1.0;
        public static final double kHighGearRatio = 1.0; //TODO: If switched to Rev Through Bore Encoder, do not need this
        //TODO: Find values for everything below
        public static final double kDriveEpsilon = 0.1; 
        public static final double kRotateEpsilon = 3.0;
        public static final double[] kLinearGains = {0.0, 0.0, 0.0};
        public static final double[] kRotationalGains = {0.0, 0.0, 0.0};
        public static final double kRampRateToMaxSpeed = 0.02;
        public static final double kMaxManualLinearAcceleration = 0.05;
        public static final double kMaxManualRotationAcceleration = 0.05;
        public static final double kThrottleDecelerationProportion = 0.8; 
        public static final double kRotationalDecelerationProportion = 0.8;

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

        public static final int kGearShiftForwardId = 4;
        public static final int kGearShiftReverseId = 5;
    }

    public static final class SuperstructureConstants {

        public static final double kDrivebaseTargetingEpsilon = 3;
        
        public static final class ShooterConstants {
            public static final int kLauncherMotorId = 7;
            //public static final int kFeederMotorId = 8;

            public static final int kLidarId = 0;

            public static final int kPidId = 0;

            public static final double kMaxOutput = 1.0;
            public static final double kMinOutput = 0.0;
            private static final double kP = 0.002;
            private static final double kI = 0.0;
            private static final double kD = 0.0;
            private static final double kFF = 0.0002;
            private static final double kIz = 0.0;
            public static final double[] kGains = {kP, kI, kD, kFF, kIz}; 
            public static final double kMaxVelocity = 5000;
            public static final double kLowAccelerationRPMPerSecond = kMaxVelocity / 4;
            public static final double kHighAccelerationRPMPerSecond = kMaxVelocity / 0.5;
            public static final double kVelocityAccelShiftThresholdRPM = 1100.0;
            public static final double kShooterSpeedEpsilon = 50.0;
            public static final double kFeederSpeed = 0.65;

            public static final double[][] kDistanceFeetToRpmLUT = {
                {0, 1000.0}, 
                {5, 1100.0},
                {10, 1200.0},
                {15, 1400.0},
                {20, 1600.0},
                {25, 1900.0},
                {30, 2300.0},
                {35, 2700.0},
                {40, 3200.0},
                {45, 3800.0}
            };

            //TODO: Consider Cubic Spline Interpolation if 2nd Degree Polynomial Regression isn't accurate enough
            // http://mathworld.wolfram.com/CubicSpline.html
            public static final PolynomialRegression kShooterPolynomial = new PolynomialRegression(kDistanceFeetToRpmLUT, 2);
        }  

        public static final class TurretConstants {
            public static final int kTurretMotorId = 11;

            public static final int kTurretEncoderDioId = 0;

            public static final double kManualControlScaleFactor = 0.25;
            public static final double[] kGains = {0.0, 0.0, 0.0};
            public static final double kTurretEpsilon = 2.0; //TODO: Find the right value
            public static final double kTurretDegreesPerEncoderRotation = 45.0; // 8 rot == 360 deg

            public static final double kMaxTx = 999.9;

            public static final double kPowerCellDiameterInches = 7.0;
            public static final double kPowerCellClearance = 2.0;

            private static final double kLowTargetWidthInches = 34;
            private static final double kHighTargetWidthInches = 39.25;
            public static final double[] kTargetWidth = {kHighTargetWidthInches, kLowTargetWidthInches};

            //TODO: Might further adjust turretSetpointInDegrees to a lower range than 180deg
            public static final double kPhysicalTurretRotationLimit = 180; 
        }

        public static final class StorageConstants { 
            public static final int kBottomBeltDriverMotorId = 10;
            public static final int kBeltDriverMotorId = 9;

            public static final int kIntakeEmitterId = 2;
            public static final int kIntakeDetectorId = 1;

            public static final int kChamberEmitter = 4;
            public static final int kChamberDetectorId = 5;

            public static final int kLidarId = 0;

            public static final double kBeltSpeed = 1.0;
            public static final double kLowerBeltSpeed = 0.44;
            public static final double kMaxEmptyStorageDistanceInches = 25.0; //TODO: Find real value

            public static final int kMedianFilterSize = 15;

            public static final double kBallInIntakeMinIn = 29.5;
            public static final double kIntakeDepthIn = 4.0;
            public static final double kBallInIntakeMaxIn = kBallInIntakeMinIn + kIntakeDepthIn;
            public static final double kBallDiameterIn = 7.0;
            public static final double kBallSpacingIn = 1.0;
            public static final double kRangeToPositionCLeadingEdgeIn = 14.0 - 2.0;
            public static final double kRangeToPositionBLeadingEdgeIn = kRangeToPositionCLeadingEdgeIn + kBallDiameterIn + kBallSpacingIn - 1.0;
            public static final double kRangeToPositionALeadingEdgeIn = kRangeToPositionBLeadingEdgeIn + kBallDiameterIn + kBallSpacingIn;
            public static final double kRangeToPositionChamberLeadingEdgeIn = 5.0;
        }

        public static final class IntakeConstants {
            public static final int kIntakeMotorId = 14;

            public static final double kIntakeSpeed = 0.5;

            //Pneumatics PCM A   
            public static final int kIntakeForwardId = 0;
            public static final int kIntakeReverseId = 1;
        }
    }

    public static final class WheelControllerConstants {
        public static final int kWheelControllerId = 15;

        // Expected to read color 90deg from the field color sensor, therefore 2 45deg color wedges over
        public static final int kRobotReadingVarianceIndex = 2;
        public static final double kWheelSpinSpeed = 0.4;

        public static final double kColorTolerance = 0.08;

        public static final int kExtenderForwardId = 6;
        public static final int kExtenderReverseId = 7;
    }

    public static final class ClimberConstants {
        public static final int kElevatorId = 12;
        public static final int kLifterId = 13;

        public static final int kElevatorPidId = 0;
        //TODO: Tune all values below
        public static final double[] kGains = {1.0,0,0}; 

        private static final double kStartEnc = 0;
        private static final double kLowEnc = 1024;
        private static final double kMedEnc = 4096;
        private static final double kHighEnc = 8192;
        public static final double[] kElevataorLevelEncoderValues = {kStartEnc, kLowEnc, kMedEnc, kHighEnc}; 

        //Below Two constants are in CTRE Mag Encoder Units Per 100Ms Per Sec
        public static final int kMagicAcceleration = 1000;
        public static final int kMagicCruiseVelocity = 5000;

        public static final int kIntegralZone = 200;
        public static final int kEpsilon = 12; 
        public static final int kTimeout = 256;

        public static final double kLifterSpeed = 1.0; //TODO: Find value
    }
}