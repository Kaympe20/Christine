package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Flywheel extends SubsystemBase {
  public TalonFX flywheel = new TalonFX(17);

  public static CANSparkMax pivot = new CANSparkMax(16, MotorType.kBrushless);

  public Flywheel() {}

  public void setAngle(double angle){
    flywheel.setPosition(angle);
  }

  public void setSpeed(double speed){
    pivot.set(speed);
  }

  public void stop(){
    pivot.stopMotor();
  }

  public static double pivotAngle(double height, double distance){
    return Math.atan(height/distance);
  }

  public static double RPM(double angle, double distance){
    double radius = 1; // TODO: find radius for the flywheel
    double conversion_factor = 60/(2*Math.PI*radius);
    double raw_rpm = conversion_factor * Math.sqrt(distance * -9.8 * Math.sin(2*angle)); // NOTE: 9.8 could be negative or positive, idk
    return Math.max(0,Math.min(6000.0, raw_rpm))/6000;
  }

  @Override
  public void periodic() {}
}