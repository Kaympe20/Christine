package frc.robot.modules;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.utility.Constants.DriveConstants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandcoder.Canandcoder;


public class KrakenSwerveModule implements SwerveModule {
    public final TalonFX driveMotor;
    public final CANSparkMax steerMotor;
    public final Canandcoder steerEncoder; 
    double desiredAngle;

    final double PI2 = 2.0 * Math.PI; 

    public KrakenSwerveModule(ShuffleboardLayout tab, int driveID, int steerID, int steerCANID){
        driveMotor = new TalonFX(driveID, "rio");
        steerMotor = new CANSparkMax(steerID, MotorType.kBrushless);
        steerEncoder = new Canandcoder(steerCANID);
        

        Canandcoder.Settings settings = new Canandcoder.Settings();
        settings.setInvertDirection(true);
        steerEncoder.clearStickyFaults();
        steerEncoder.resetFactoryDefaults(false);
        steerEncoder.setSettings(settings);

        TalonFXConfiguration config = new TalonFXConfiguration();
        
        steerMotor.setSmartCurrentLimit(20);
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        steerMotor.setIdleMode(IdleMode.kBrake);
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;


        steerMotor.getEncoder().setPositionConversionFactor(Math.PI * DriveConstants.STEER_REDUCTION);
        steerMotor.getEncoder().setVelocityConversionFactor(Math.PI * DriveConstants.STEER_REDUCTION / 60);
        steerMotor.getEncoder().setPosition(steerAngle());
        
        driveMotor.setInverted(true);
        steerMotor.setInverted(false);

        // driveMotor.enableVoltageCompensation(12);
        driveMotor.getConfigurator().apply(config);

        steerMotor.getPIDController().setP(0.1);
        steerMotor.getPIDController().setI(0.0);
        steerMotor.getPIDController().setD(1.0);

        steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0,100);
        steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1,20);
        steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2,20);

        tab.addDouble("Absolute Angle", () -> Math.toDegrees(steerAngle())); 
        tab.addDouble("Current Angle", () -> Math.toDegrees(steerMotor.getEncoder().getPosition()));
        tab.addDouble("Target Angle", () -> Math.toDegrees(desiredAngle));
        tab.addBoolean("Active", steerEncoder::isConnected);
    }

    public void resetDrivePosition() {
        driveMotor.setPosition(0.0);
    }

    public void resetSteerPosition(){
        steerMotor.getEncoder().setPosition(steerAngle());
    }

    public void resetAbsolute(){
        steerEncoder.setAbsPosition(0,250);
    }

    public double drivePosition(){
        //return driveMotor.getPosition().getValue() * 0.01 * ((DriveConstants.WHEEL_DIAMETER * Math.PI) / DriveConstants.DRIVE_REDUCTION);
        return driveMotor.getPosition().getValue() * .502 * DriveConstants.WHEEL_DIAMETER;
    }

    public double steerAngle(){
        return (steerEncoder.getAbsPosition() * PI2) % PI2;
    }

    public void stop(){
        driveMotor.stopMotor();
        steerMotor.stopMotor();
    }

    public void set(double driveVolts, double targetAngle){
        resetSteerPosition();
        
        // TODO: Check if the values we pass are even able to be negative

        // Put in range of [0, 2Pi)
        targetAngle %= PI2;
        targetAngle += (targetAngle < 0.0) ? PI2 : 0.0;

        desiredAngle = targetAngle;

        double diff = targetAngle - steerAngle();

        // TODO: check how the PID handles 0 -> 135 as the unmodified angle (335 as the modified angle)
        if (diff > (Math.PI/2.0) || diff < -(Math.PI/2.0)){ // move to a closer angle and drive backwards 
            targetAngle = (targetAngle + Math.PI) % PI2; 
            driveVolts *= -1.0;
        }

        driveMotor.setVoltage(driveVolts);
        steerMotor.getPIDController().setReference(targetAngle, ControlType.kPosition);
    }

}