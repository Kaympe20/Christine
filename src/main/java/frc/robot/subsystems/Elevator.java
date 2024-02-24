// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
   public TalonFX elevatorMotor = new TalonFX(19, "rio");
   public Follower elevatorMotorFollower = new Follower(elevatorMotor.getDeviceID(), true);

   public Elevator() {
    
  }
 
  public void setSpeed(double volts){
    elevatorMotor.setVoltage(volts);
  }

  public void setPosition(double position){
    elevatorMotor.setPosition(position);
  }

  public void stop(){
    elevatorMotor.stopMotor();
  }

  public void resetOffset(){
  }

  @Override
  public void periodic() {

  }
}