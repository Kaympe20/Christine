// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.utility.IO;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoFiring extends SequentialCommandGroup {
  /** Creates a new AutoFiring. */
  public AutoFiring(IO io) {
    addRequirements(io.intake, io.shooter, io.limelight);
    addCommands(
        new RepeatCommand(
            new SequentialCommandGroup(
                new InstantCommand(() -> io.shooter.flywheelVoltage(-16)),
                new PassOff(io),
                new WaitUntilCommand(
                    () -> io.limelight.distance() < 3 && Math.abs(io.limelight.targetData().horizontalOffset) < 10),
                new CloseUpShooting(io))));
  }
}
