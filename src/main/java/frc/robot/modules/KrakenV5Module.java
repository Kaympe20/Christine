// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.modules;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.reduxrobotics.sensors.canandcoder.Canandcoder;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.utility.Constants.DriveConstants;

public class KrakenV5Module {
    public TalonFX driveMotor;
    public CANSparkMax steerMotor;
    public Canandcoder steerEncoder;

    final double PI2 = 2 * Math.PI;

    double desiredAngle;

    public KrakenV5Module(ShuffleboardLayout tab, int driveID, int steerID, int steerEncoderID, double offset) {
        driveMotor = new TalonFX(driveID);
        steerMotor = new CANSparkMax(steerID, MotorType.kBrushless);
        steerEncoder = new Canandcoder(steerEncoderID);

        Canandcoder.Settings settings = new Canandcoder.Settings();
        settings.setInvertDirection(true);

        steerEncoder.clearStickyFaults();
        steerEncoder.resetFactoryDefaults(false);
        steerEncoder.setSettings(settings);

        driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 0, 0));

        steerMotor.setSmartCurrentLimit(20);

        steerMotor.setIdleMode(IdleMode.kBrake);
        driveMotor.setNeutralMode(NeutralMode.Coast);

        // NOTE: drive motor positoin velocity conversion factor is Missing

        steerMotor.getEncoder().setPositionConversionFactor(Math.PI * DriveConstants.STEER_REDUCTION);
        steerMotor.getEncoder().setVelocityConversionFactor(Math.PI * DriveConstants.STEER_REDUCTION / 60);
        steerMotor.getEncoder().setPosition(steerAngle());
        
        driveMotor.setInverted(true);
        steerMotor.setInverted(false);

        driveMotor.configVoltageCompSaturation(12);
        driveMotor.enableVoltageCompensation(true);

        steerMotor.getPIDController().setP(0.1);
        steerMotor.getPIDController().setI(0);
        steerMotor.getPIDController().setD(1);

        steerMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 100);
        steerMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 20);
        steerMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 20);

        tab.addDouble("Absolute Angle", () -> Math.toDegrees(steerAngle())); 
        tab.addDouble("Current Angle", () -> Math.toDegrees(steerMotor.getEncoder().getPosition()));
        tab.addDouble("Target Angle", () -> Math.toDegrees(desiredAngle));
        tab.addBoolean("Active", steerEncoder::isConnected);
    
    }
        public void resetDrivePosition() {
            driveMotor.setSelectedSensorPosition(0.0);
        }

        public void resetSteerPosition() {
            driveMotor.setSelectedSensorPosition(steerAngle());
        }

        public void resetAbsolute() {
            steerEncoder.setAbsPosition(0.0, 250);
        }

        public double drivePosition() {
            return driveMotor.getSelectedSensorPosition();
        }

        public double steerAngle() {
            return (steerEncoder.getAbsPosition() * PI2) % PI2;
        }

        public void set(double driveVolts, double targetAngle) {
            resetSteerPosition();

            targetAngle %= PI2;
            targetAngle += (targetAngle < 0.0) ? PI2 : 0.0;

            desiredAngle = targetAngle;

            double diff = targetAngle - steerAngle();

            if (diff > (Math.PI / 2.0) || diff < -(Math.PI / 2.0)) {
                targetAngle = (targetAngle + Math.PI) % PI2;
                driveVolts *= -1;
            }

            driveMotor.set(ControlMode.PercentOutput, driveVolts);
            steerMotor.getPIDController().setReference(targetAngle, ControlType.kPosition);
        }
    
    
}
