// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  public CANSparkMax pivot = new CANSparkMax(10, MotorType.kBrushless);
  public TalonFX intake = new TalonFX(11, "rio");
  public DigitalInput beamBreak = new DigitalInput(0);
  public DutyCycleEncoder encoder = new DutyCycleEncoder(2);
  public boolean closed;

  public final double closedAngle = 80.0;

  public Intake() {
    pivot.setIdleMode(IdleMode.kCoast);
    pivot.setSmartCurrentLimit(20);
    closed = (angle() < closedAngle);
  }

  public void speed(double speed) {
    intake.set(speed);
  }

  public void pivotVoltage(double volts) {
    pivot.setVoltage(volts);
  }

  public void intakeVoltage(double voltage) {
    intake.setVoltage(voltage);
  }

  public double angle() {
    return encoder.getAbsolutePosition() * 360;
  }

  public boolean loaded() {
    return beamBreak.get();
  }

  @Override
  public void periodic() {
    closed = (angle() < closedAngle);
    SmartDashboard.putBoolean("Beam Break", loaded());
    SmartDashboard.putNumber("Intake Angle", angle());
    SmartDashboard.putBoolean("Intake Closed", closed);
  }
}