// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
//import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {
   public CANSparkMax pivot = new CANSparkMax(10, MotorType.kBrushless);
   public TalonFX intake = new TalonFX(11, "rio");
   public DigitalInput beam_break = new DigitalInput(0);
   public DutyCycleEncoder encoder = new DutyCycleEncoder(2);
   public boolean closed;

  public Intake() {
    pivot.getPIDController().setP(0.3);
    pivot.getPIDController().setI(0);
    pivot.getPIDController().setD(0);
    
    pivot.setIdleMode(IdleMode.kBrake);
    encoder.setPositionOffset(0.0);
    pivot.getPIDController().setPositionPIDWrappingEnabled(true);
    pivot.getPIDController().setPositionPIDWrappingMaxInput(Math.PI * 2);
    
    closed = (angle() < 93);
  }

  public void toggle(){
      pivot.setVoltage( 5 * ( closed ? -1 : 1) );
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
    encoder.setPositionOffset(0);
  
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Beam Break", loaded());
    SmartDashboard.putNumber("Intake Angle", angle());
    SmartDashboard.putBoolean("Intake Closed", closed);
    //SmartDashboard.putNumber("Intake Absolute Angle", encoder.getAbsolutePosition());
  }
}