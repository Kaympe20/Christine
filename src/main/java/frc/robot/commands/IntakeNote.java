// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.utility.IO;

public class IntakeNote extends Command {
  
  IO io;
  boolean auton = false;

  public IntakeNote(IO io, boolean auton) {
    this.io = io;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    io.intake.intakeVoltage(12);
  }

  @Override
  public void end(boolean interrupted) {
    if (auton)
    new WaitCommand(.025).andThen(new InstantCommand(() -> io.intake.speed(0))).schedule();
    else io.intake.speed(0);
  }

  @Override
  public boolean isFinished() {
    return !io.intake.loaded();
  }
}
