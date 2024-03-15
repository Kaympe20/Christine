package frc.robot.subsystems;

//import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Flywheel extends SubsystemBase {

  public TalonFX pivot = new TalonFX(13, "rio");
  public TalonFX flywheel = new TalonFX(15, "rio");
  public DutyCycleEncoder encoder = new DutyCycleEncoder(4);
  public CANSparkMax helper = new CANSparkMax(12, MotorType.kBrushless);

  public final double PASS_OFF_ANGLE = 75.0;
  public final double AMP = 163.0;

  public Flywheel() {
    helper.setSmartCurrentLimit(20);
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 0.3;
    configs.Slot0.kI = 0;
    configs.Slot0.kD = 0.01;
    flywheel.getConfigurator().apply(configs);
  }

  public double angle() {
    return encoder.getAbsolutePosition() * 360;
  }

  public void pivotVoltage(double volts) {
    pivot.setVoltage(volts);
  }

  public void flywheelVoltage(double volts) {
    flywheel.setVoltage(volts);
  }

  public void helperVoltage(double volts) {
    helper.setVoltage(volts);
  }

  public void flywheelSpeed(double speed) {
    flywheel.set(speed);
  }

  public void setRPM(double RPM) {
    double flywheelRPM = Math.max(Math.min(RPM, 6000), 0) / 6000;
    flywheel.set(RPM / 6000);
  }

  public void setVelocity(double velocity) {
    flywheel.set(velocity / 100);
  }

  public static double estimateAngle(double height, double distance) {
    return Math.toDegrees(Math.atan(height / distance));
  }

  // public static double RPM(double angle, double distance) {
  //   double radius = Units.inchesToMeters(1.5);
  //   double conversion_factor = 60 / (2 * Math.PI * radius);
  //   double raw_rpm = conversion_factor * Math.sqrt(distance * 9.8 * Math.sin(2 * angle)); // NOTE: 9.8 could be negative
  //                                                                                         // or positive, idk
  //   return Math.max(Math.min(raw_rpm, 6000), 0) / 6000;
  // }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Flywheel Angle", angle());
    SmartDashboard.putNumber("Flywheel RPM", flywheel.getVelocity().getValueAsDouble());
  }
}