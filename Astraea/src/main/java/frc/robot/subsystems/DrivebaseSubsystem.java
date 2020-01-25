package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivebaseSubsystem extends SubsystemBase {

    private CANSparkMax drivebaseMotorPortA;
    private CANSparkMax drivebaseMotorPortB;
    private CANSparkMax drivebaseMotorPortC;
    private SpeedControllerGroup speedControllerGroupPortSide;
    private CANSparkMax drivebaseMotorStarboardA;
    private CANSparkMax drivebaseMotorStarboardB;
    private CANSparkMax drivebaseMotorStarboardC;
    private SpeedControllerGroup speedControllerGroupStartbordSide;
    private DifferentialDrive differentialDrive;
    private DoubleSolenoid gearShiftDoubleSolenoid;

    private final double kRegularMaxSpeed = 1.0;
    private final double kPrecisionMaxSpeed = 0.5; //TODO: Find a good value for this
    private double mSelectedMaxSpeedProportion;
    public DrivebaseSubsystem() {

        drivebaseMotorPortA = new CANSparkMax(Constants.kMotorControllerPortA, MotorType.kBrushless);
        //addChild("Drivebase Motor Port A",drivebaseMotorPortA);
        drivebaseMotorPortA.setInverted(false);
                
        drivebaseMotorPortB = new CANSparkMax(Constants.kMotorControllerPortB, MotorType.kBrushless);
        //addChild("Drivebase Motor Port B",drivebaseMotorPortB);
        drivebaseMotorPortB.setInverted(false);
                
        drivebaseMotorPortC = new CANSparkMax(Constants.kMotorControllerPortC, MotorType.kBrushless);
        //addChild("Drivebase Motor Port C",drivebaseMotorPortC);
        drivebaseMotorPortC.setInverted(false);
                
        speedControllerGroupPortSide = new SpeedControllerGroup(drivebaseMotorPortA, drivebaseMotorPortB, drivebaseMotorPortC);
        addChild("Speed Controller Group Port Side",speedControllerGroupPortSide);
                
        drivebaseMotorStarboardA = new CANSparkMax(Constants.kMotorControllerStarboardA, MotorType.kBrushless);
        //addChild("Drivebase Motor Starboard A",drivebaseMotorStarboardA);
        drivebaseMotorStarboardA.setInverted(true);
                
        drivebaseMotorStarboardB = new CANSparkMax(Constants.kMotorControllerStarboardB, MotorType.kBrushless);
        //addChild("Drivebase Motor Starboard B",drivebaseMotorStarboardB);
        drivebaseMotorStarboardB.setInverted(true);
                
        drivebaseMotorStarboardC = new CANSparkMax(Constants.kMotorControllerStarboardC, MotorType.kBrushless);
        //addChild("Drivebase Motor Starboard C",drivebaseMotorStarboardC);
        drivebaseMotorStarboardC.setInverted(true);
        
        speedControllerGroupStartbordSide = new SpeedControllerGroup(drivebaseMotorStarboardA, drivebaseMotorStarboardB, drivebaseMotorStarboardC  );
        //addChild("Speed Controller Group Startbord Side",speedControllerGroupStartbordSide);
                
        differentialDrive = new DifferentialDrive(speedControllerGroupPortSide, speedControllerGroupStartbordSide);
        addChild("Differential Drive",differentialDrive);
        differentialDrive.setSafetyEnabled(true);
        differentialDrive.setExpiration(0.1);
        differentialDrive.setMaxOutput(1.0);
                
        gearShiftDoubleSolenoid = new DoubleSolenoid(Constants.kPcmId, Constants.kGearShifDoubleSolenoidChannelForward, Constants.kGearShifDoubleSolenoidChannelReverse);
        addChild("Gear Shift Port Double Solenoid",gearShiftDoubleSolenoid);    

        mSelectedMaxSpeedProportion = kRegularMaxSpeed;
    }

    @Override
    public void periodic() {
    }
    
    public void manualDrive(XboxController theXboxController) {
        differentialDrive.curvatureDrive(theXboxController.getY(Hand.kLeft) * mSelectedMaxSpeedProportion, theXboxController.getX(Hand.kRight) * mSelectedMaxSpeedProportion, true);
    }

    public void togglePrecisionDriving() {
        if(mSelectedMaxSpeedProportion == kRegularMaxSpeed) {
            mSelectedMaxSpeedProportion = kPrecisionMaxSpeed;
        } else {
            mSelectedMaxSpeedProportion = kRegularMaxSpeed;
        }
    }

    public void toggleDrivebaseGearing(){
        switch (gearShiftDoubleSolenoid.get()) {
            case kForward : 
                gearShiftDoubleSolenoid.set(Value.kReverse);
                break;
            case kReverse : 
                gearShiftDoubleSolenoid.set(Value.kForward);
                break;
            default :
                gearShiftDoubleSolenoid.set(Value.kReverse);
                System.out.println("Error: Gear shift toggle invalid value.");
                break;
        }
    }

}

