// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.utility.IO;

public class Endgame extends SequentialCommandGroup {
  public Endgame(IO io) {
    addRequirements(io.shooter, io.climber, io.intake);
    addCommands(
        new InstantCommand(() -> io.climber.setHangPos(io.climber.HANG_UP_POS)),
        new InstantCommand(() -> io.climber.setHangPos(io.climber.HANG_DOWN_POS)),
        new InstantCommand(() -> io.climber.setElevatorPos(io.climber.ELEVATOR_UP_POS)),
        new AmpShooting(io));
  }
}
