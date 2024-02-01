// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
   public CANSparkMax pivot = new CANSparkMax(5, MotorType.kBrushless); // TODO: Replace ID
   public CANSparkMax intake = new CANSparkMax(6, MotorType.kBrushless); // TODO: Replace ID
   public DigitalInput beam_break = new DigitalInput(0); // TODO: Replace ID
   public boolean intakeOpen;

  public Intake() {
    pivot.getPIDController().setP(0.3);
    pivot.getPIDController().setI(0);
    pivot.getPIDController().setD(0);
    pivot.getEncoder().setPosition(0);
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

  public void stop(){
    intake.stopMotor();
  }

  public boolean loaded() {
    return beam_break.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Beam Break", loaded());
  }
}