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


public class CloseUpShooting extends SequentialCommandGroup {

  IO io;

  public CloseUpShooting(IO io) {
    addRequirements(io.shooter, io.intake);
    ProfiledShooter profiledShoot = new ProfiledShooter(io, io.shooter.PASS_OFF_ANGLE);
    addCommands(new ParallelRaceGroup(profiledShoot,
        new SequentialCommandGroup(
            new InstantCommand(() -> profiledShoot.setAngle(io.shooter.PASS_OFF_ANGLE)),
            new InstantCommand(() -> io.profiledShoot.setAngle(io.shooter.PASS_OFF_ANGLE)),
            new WaitCommand(1.0),
            new InstantCommand(() -> io.shooter.flywheelVoltage(-16)),
            new WaitCommand(0.5),
            new InstantCommand(() -> io.shooter.helperVoltage(-6)),
            new InstantCommand(() -> io.intake.speed(-1)),
            new WaitCommand(0.3),
            new InstantCommand(() -> io.shooter.flywheelVoltage(0.0)),
            new InstantCommand(() -> io.shooter.helperVoltage(0.0)),
            new InstantCommand(() -> io.intake.speed(0.0)))));
  }
}