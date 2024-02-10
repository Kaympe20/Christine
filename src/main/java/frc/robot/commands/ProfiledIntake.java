// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utility.IO;

public class ProfiledIntake extends Command {
  PIDController controller = new PIDController(0.3, 0, 0);
  Constraints constraints = new Constraints(2, 2);
  TrapezoidProfile profile = new TrapezoidProfile(constraints);
  Timer time = new Timer();
  double targetAngle;
  IO io;

  boolean stopped;

  public ProfiledIntake(IO io, double init_angle) {
    this.io = io;
    targetAngle = init_angle;
    controller.reset();
    addRequirements(io.intake);
  }

  public void setAngle(double angle) {
    controller.reset();
    time.restart();
    targetAngle = angle;
    stopped = false;
  }

  public void stop() {
    io.intake.intakeVolts(0);
    stopped = true;
  }

  @Override
  public void execute() {
    if (!stopped) {
      State out = profile.calculate(time.get(), new State(io.intake.angle(), 0.0), new State(targetAngle, 0));
      double output = controller.calculate(io.intake.angle(), out.position);
      SmartDashboard.putNumber("Expected Profile Angle", out.position);
      SmartDashboard.putNumber("Angle Voltage", output);

      io.intake.intakeVolts(output);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
