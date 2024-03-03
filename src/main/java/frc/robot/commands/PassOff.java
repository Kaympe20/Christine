// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.utility.IO;

public class PassOff extends SequentialCommandGroup {

  public PassOff(IO io, ProfiledShooter shoot) {
    ProfiledShooter profiledShoot = new ProfiledShooter(io, 70.0);
    addCommands(
        new ParallelDeadlineGroup(profiledShoot,
            new SequentialCommandGroup(
                new InstantCommand(() -> profiledShoot.setAngle(70)),
                new ConditionalCommand(
                  new SequentialCommandGroup(
                    new ToggleIntake(io),
                    new IntakeNote(io),
                    new ToggleIntake(io)), 
                  new InstantCommand(() -> profiledShoot.setAngle(70.0)),
                    () -> io.intake.closed))));
  }
}
