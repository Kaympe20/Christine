package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Flywheel extends SubsystemBase {

  public TalonFX pivot = new TalonFX(13, "rio");
  public TalonFX flywheel = new TalonFX(15, "rio");
  public DutyCycleEncoder encoder = new DutyCycleEncoder(3);
  public CANSparkMax helperMotor = new CANSparkMax(12, MotorType.kBrushless);

  public final double PASS_OFF_ANGLE = 63.0; // 54.
  public final double AMP = 163.0;

  public Flywheel() {

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.ClosedLoopGeneral.ContinuousWrap = false;
    config.Slot0.kP = 0.3;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

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
    return Math.toDegrees(Math.atan(height/distance));
  }

  public static double RPM(double angle, double distance){
    double radius = Units.inchesToMeters(1.5);
    double conversion_factor = 60/(2*Math.PI*radius);
    double raw_rpm = conversion_factor * Math.sqrt(distance * 9.8 * Math.sin(2*angle)); // NOTE: 9.8 could be negative or positive, idk
    return Math.max(Math.min(raw_rpm,6000),0)/6000;
    // return raw_rpm;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Flywheel Angle", getPivotAngle());
    SmartDashboard.putNumber("Flywheel RPM", flywheel.getVelocity().getValueAsDouble());
  }
}