// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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
import frc.robot.utility.DebugTable;

public class Climber extends SubsystemBase {
  public CANSparkMax elevatorMotor = new CANSparkMax(19, MotorType.kBrushless);
  public CANSparkMax elevatorFollower = new CANSparkMax(20, MotorType.kBrushless);

  public TalonFX hangMotor = new TalonFX(21, "rio");
  public TalonFX hangFollower = new TalonFX(22, "rio");

  public final static double ELEVATOR_DOWN_POS = 0;
  public final static double ELEVATOR_UP_POS = 240; //TODO: PLACEHOLDER

  public final static double HANG_DOWN_POS = 0;
  public final static double HANG_UP_POS = 145; //TODO: PLACEHOLDER

  public Climber() {
    elevatorFollower.follow(elevatorMotor, true);
    hangFollower.setControl(new Follower(hangMotor.getDeviceID(), true));

    elevatorMotor.setIdleMode(IdleMode.kBrake);
    elevatorFollower.setIdleMode(IdleMode.kBrake);
    hangMotor.setNeutralMode(NeutralModeValue.Brake);
    hangFollower.setNeutralMode(NeutralModeValue.Brake);

    elevatorMotor.setSmartCurrentLimit(35);
    elevatorFollower.setSmartCurrentLimit(35);

    elevatorMotor.getPIDController().setP((double) DebugTable.get("Elevator Kp", 1.0));
    elevatorMotor.getPIDController().setI(0);
    elevatorMotor.getPIDController().setD(0);
    elevatorMotor.getPIDController().setSmartMotionAllowedClosedLoopError(10, 0);


    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 1;
    configs.Slot0.kI = 0;
    configs.Slot0.kD = 0;
    configs.CurrentLimits = new CurrentLimitsConfigs().withStatorCurrentLimitEnable(true).withStatorCurrentLimit(20);
    hangMotor.getConfigurator().apply(configs);
    hangFollower.getConfigurator().apply(configs);
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
    elevatorFollower.getPIDController().setReference(pos, ControlType.kPosition);
  }

  public void setHangVolts(double volts) {
    hangMotor.setVoltage(volts);
  }

  public double hangError(){
    return hangMotor.getClosedLoopError().getValue();
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

  public void resetEncoders(){
    hangMotor.setPosition(0);
    elevatorMotor.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hang Pos", getHangPos());
    SmartDashboard.putNumber("Elevator Pos", getElevatorPos());
  }
}