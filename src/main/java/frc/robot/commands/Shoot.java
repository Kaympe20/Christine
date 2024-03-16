// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.utility.IO;

public class Shoot extends SequentialCommandGroup {

  public Shoot(IO io) {
    ProfiledShooter profiledShoot = new ProfiledShooter(io, io.shooter.PASS_OFF_ANGLE);
    addCommands(new ParallelRaceGroup(profiledShoot,
        new SequentialCommandGroup(
            new InstantCommand(() -> io.profiledShoot.setAngle(io.shooter.PASS_OFF_ANGLE)),
            new InstantCommand(() -> profiledShoot.setAngle(io.shooter.PASS_OFF_ANGLE)),
            new WaitCommand(0.1),
            new WaitUntilCommand(() -> Math.abs(profiledShoot.controller.getPositionError()) < 2),
            new InstantCommand(() -> io.shooter.helperVoltage(-12)),
            new InstantCommand(() -> io.intake.speed(-1)),
            new WaitCommand(0.2),
            new InstantCommand(() -> io.shooter.helperVoltage(0.0)),
            new InstantCommand(() -> io.intake.speed(0.0)))));
  }
}
