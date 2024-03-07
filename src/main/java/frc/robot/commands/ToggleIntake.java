// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utility.IO;

public class ToggleIntake extends Command {

  IO io;

  boolean origin;

  final double MAX_VOLTS = 6;

  public  ToggleIntake(IO io) {
    this.io = io;
    addRequirements(io.intake);
  }

  @Override
  public void initialize() {
        origin = (io.intake.angle() < io.intake.closedAngle);
    io.intake.pivot.setVoltage( MAX_VOLTS * ( origin ? -1 : 1) );

  }

  @Override
  public void execute() {
    io.intake.closed = (io.intake.angle() < io.intake.closedAngle);
  }

  @Override
  public void end(boolean interrupted) {
    io.intake.pivotVoltage(0);
  }

  @Override
  public boolean isFinished() {
    return origin != io.intake.closed;
  }
}
