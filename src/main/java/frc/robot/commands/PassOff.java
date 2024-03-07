// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.utility.IO;

public class PassOff extends SequentialCommandGroup {

  public PassOff(IO io) {
    ProfiledShooter profiledShoot = new ProfiledShooter(io, io.shooter.PASS_OFF_ANGLE);
    addRequirements(io.intake, io.shooter);
    addCommands(
        new ParallelRaceGroup(profiledShoot,
            new SequentialCommandGroup(
                new InstantCommand(() -> profiledShoot.setAngle(io.shooter.PASS_OFF_ANGLE)),
                new WaitCommand(0.75),
                new ConditionalCommand(
                    new SequentialCommandGroup(
                        new ToggleIntake(io),
                        new IntakeNote(io),
                        new ToggleIntake(io)),
                    new SequentialCommandGroup(
                        new IntakeNote(io),
                        new ToggleIntake(io),
                        new InstantCommand(() -> io.profiledShoot.setAngle(io.shooter.PASS_OFF_ANGLE))),
                    () -> io.intake.closed)))       );
  }

}
