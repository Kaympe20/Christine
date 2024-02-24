package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Flywheel extends SubsystemBase {
  public TalonFX flywheel = new TalonFX(17);
  public static TalonFX pivot = new TalonFX(16);
  public CANSparkMax helperMotor = new CANSparkMax(18, MotorType.kBrushless);
  public DutyCycleEncoder encoder = new DutyCycleEncoder(3); //TODO: Set ID when added


  public Flywheel() {

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.ClosedLoopGeneral.ContinuousWrap = true;
    config.Slot0.kP = 0.3;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    pivot.getConfigurator().apply(config);
  }

  public void setAngle(double angle){
    flywheel.setPosition(angle);
  }

  public double getPivotAngle() {
    return encoder.getAbsolutePosition() * 360;
  }

  public void setPivotVoltage(double volts) {
    pivot.setVoltage(volts);
  }

  public void setFlywheelVoltage(double volts) {
    flywheel.setVoltage(volts);
  }

  public void setHelperVoltage(double volts) {
    helperMotor.setVoltage(volts);
  }

  public void setSpeed(double speed){
    pivot.set(speed);
    helperMotor.setVoltage(flywheel.get());
  }

  public void pivotStop(){
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