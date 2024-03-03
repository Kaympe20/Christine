// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utility.IO;

public class IntakeNote extends Command {
  
  IO io;

  public IntakeNote(IO io) {
    this.io = io;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    io.intake.intakeVoltage(11);
  }

  @Override
  public void end(boolean interrupted) {
    io.intake.speed(0);
  }

  @Override
  public boolean isFinished() {
    return !io.intake.loaded();
  }
}
