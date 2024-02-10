// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeNeo extends SubsystemBase {
   public CANSparkMax pivot = new CANSparkMax(14, MotorType.kBrushless);
   public CANSparkMax intake = new CANSparkMax(15, MotorType.kBrushless);
   public DigitalInput beam_break = new DigitalInput(0);
   public DutyCycleEncoder encoder = new DutyCycleEncoder(2);
   public boolean intakeOpen;

  public IntakeNeo() {
    pivot.getPIDController().setP(0.3);
    pivot.getPIDController().setI(0);
    pivot.getPIDController().setD(0);
    pivot.getEncoder().setPosition(0);
    pivot.setIdleMode(IdleMode.kBrake);
  }
 
  public void open(){
    pivot.getPIDController().setReference(0, ControlType.kPosition); //placeholder value
    intakeOpen = true;
  }

  public void close(){
    pivot.getPIDController().setReference(0, ControlType.kPosition);
    intakeOpen = false;
  }

  public void toggle(){
    if (intakeOpen) close();
    else open();
  }

  public void setSpeed(double speed){
    intake.set(speed);
  }

  public void intakeVolts(double volts){
    pivot.setVoltage(volts);
  }
  
  public void setVoltage(double voltage){
    intake.setVoltage(voltage);
  }

  public double angle(){
    return encoder.getAbsolutePosition() * 360;
  }

  public void stop(){
    intake.stopMotor();
  }

  public boolean loaded() {
    return beam_break.get();
  }

  public void resetOffset(){
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Beam Break", loaded());
    SmartDashboard.putNumber("Intake Angle", angle());
    SmartDashboard.putNumber("Intake Absolute Angle", encoder.getAbsolutePosition());
  }
}