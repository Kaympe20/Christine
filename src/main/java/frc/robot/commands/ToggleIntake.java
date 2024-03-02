// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utility.IO;

public class ToggleIntake extends Command {

  IO io;

  public ToggleIntake(IO io) {
    this.io = io;
    addRequirements(io.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    io.intake.toggle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    io.intake.stop();
    io.intake.closed = (io.intake.angle() < 93);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (io.intake.angle() > 260 && io.intake.closed ) || (io.intake.angle() < 93 && !io.intake.closed);
  }
}
