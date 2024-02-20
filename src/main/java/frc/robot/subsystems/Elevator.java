// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
   public CANSparkMax elevatorMotor = new CANSparkMax(19, MotorType.kBrushless);
   public CANSparkMax elevatorMotorFollower = new CANSparkMax(20, MotorType.kBrushless);

   public Elevator() {
    elevatorMotorFollower.follow(elevatorMotor, true);
    elevatorMotor.getPIDController().setP(0.3);
    elevatorMotor.getPIDController().setI(0);
    elevatorMotor.getPIDController().setD(0);
    elevatorMotor.getEncoder().setPosition(0);
  }
 
  public void setSpeed(double volts){
    elevatorMotor.setVoltage(volts);
  }

  public void setPosition(double position){
    elevatorMotor.getPIDController().setReference(position, ControlType.kPosition);
  }

  public void stop(){
    elevatorMotor.stopMotor();
  }

  public void resetOffset(){
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Position", elevatorMotor.getEncoder().getPosition());
  }
}