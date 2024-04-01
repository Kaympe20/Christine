// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.utility.IO;

public class PassOff extends SequentialCommandGroup {

    public PassOff(IO io, boolean auton) {
        addRequirements(io.intake, io.shooter);
        addCommands(
            new ConditionalCommand(
                    new SequentialCommandGroup(
                            new ToggleIntake(io),
                            new IntakeNote(io, auton),
                            new ToggleIntake(io)),
                    new SequentialCommandGroup(
                            new IntakeNote(io, auton),
                            new ToggleIntake(io)),
                    () -> io.intake.closed),
            new InstantCommand(io.profiledShoot::stop));
    }

}
