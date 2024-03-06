// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.utility.IO;

public class CloseUpShooting extends SequentialCommandGroup {

  IO io;

  public CloseUpShooting(IO io) {
    addRequirements(io.intake, io.shooter);
    addCommands(
            new InstantCommand(() -> io.profiledShoot.setAngle(io.shooter.PASS_OFF_ANGLE)),
            new InstantCommand(() -> io.shooter.flywheelVoltage(-16)),
            new InstantCommand(() -> io.shooter.helperVoltage(-6)),
            new WaitCommand(0.75),
            new WaitUntilCommand(() -> Math.abs(io.profiledShoot.controller.getPositionError()) < 2),
            new InstantCommand(() -> io.intake.speed(-1)),
            new WaitCommand(0.3),
            new InstantCommand(() -> io.shooter.flywheelVoltage(0.0)),
            new InstantCommand(() -> io.shooter.helperVoltage(0.0)),
            new InstantCommand(() -> io.intake.speed(0.0))
            );
  }
}
