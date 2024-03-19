// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  public CANSparkMax elevatorMotor = new CANSparkMax(19, MotorType.kBrushless);
  public CANSparkMax elevatorFollower = new CANSparkMax(20, MotorType.kBrushless);

  public TalonFX hangMotor = new TalonFX(21, "rio");
  public TalonFX hangFollower = new TalonFX(22, "rio");

  final double ELEVATOR_DOWN_POS = 0;
  final double ELEVATOR_UP_POS = 50; //TODO: PLACEHOLDER

  final double HANG_DOWN_POS = 0;
  final double HANG_UP_POS = 50; //TODO: PLACEHOLDER

  public Climber() {
    elevatorMotor.restoreFactoryDefaults();
    elevatorFollower.restoreFactoryDefaults();

    elevatorFollower.follow(elevatorMotor, true);
    hangFollower.setControl(new Follower(elevatorMotor.getDeviceId(), true));

    elevatorMotor.setIdleMode(IdleMode.kBrake);
    elevatorFollower.setIdleMode(IdleMode.kBrake);
    hangMotor.setNeutralMode(NeutralModeValue.Brake);
    hangFollower.setNeutralMode(NeutralModeValue.Brake);

    elevatorMotor.getPIDController().setP(1);
    elevatorMotor.getPIDController().setI(0);
    elevatorMotor.getPIDController().setD(0);

    elevatorFollower.getPIDController().setP(1);
    elevatorFollower.getPIDController().setI(0);
    elevatorFollower.getPIDController().setD(0);

    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 1;
    configs.Slot0.kI = 0;
    configs.Slot0.kD = 0;
    hangMotor.getConfigurator().apply(configs);
    // hangFollower.getConfigurator().apply(configs); i wanna see if we need this or not since its follower
  }
 
  public void setElevatorVolts(double volts){
    elevatorMotor.setVoltage(volts);
  }

  public void setElevatorSpeed(double speed) {
    elevatorMotor.set(speed);
  }

  public void elevatorStop() {
    elevatorMotor.stopMotor();
  }

  public double getElevatorPos() {
    return elevatorMotor.getEncoder().getPosition();
  }

  public void setElevatorPos(double pos) {
    elevatorMotor.getPIDController().setReference(pos, ControlType.kPosition);
  }

  public void setHangVolts(double volts) {
    hangMotor.setVoltage(volts);
  }

  public void setHangSpeed(double speed) {
    hangMotor.set(speed);
  }

  public void hangStop() {
    hangMotor.stopMotor();
  }

  public double getHangPos() {
    return hangMotor.getPosition().getValueAsDouble();
  }

  public void setHangPos(double pos) {
    hangMotor.setControl(new PositionVoltage(pos).withSlot(0));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hang Pos", getHangPos());
    SmartDashboard.putNumber("Elevator Pos", getElevatorPos());
  }
}