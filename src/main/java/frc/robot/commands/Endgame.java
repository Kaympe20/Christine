// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.utility.IO;

public class Endgame extends SequentialCommandGroup {

  final double HANG_DISTANCE = 3;

  public Endgame(IO io) {
    addRequirements(io.shooter, io.climber, io.chassis);
    ProfiledShooter profiledShoot = new ProfiledShooter(io, io.shooter.PASS_OFF_ANGLE);
    addCommands(
        new ParallelRaceGroup(profiledShoot,
            new SequentialCommandGroup(
                new Aimbot(io),
                new DistanceDrive(io, HANG_DISTANCE),
                new InstantCommand(() -> io.climber.setHangPos(io.climber.HANG_UP_POS)),
                new DefaultDrive(io, new ChassisSpeeds(1, 0, 0)), // kinda placeholder but just like move forward lil
                new InstantCommand(() -> io.climber.setHangPos(io.climber.HANG_DOWN_POS)),
                new InstantCommand(() -> io.climber.setElevatorPos(io.climber.ELEVATOR_UP_POS)),
                new AmpShooting(io))));
  }
}
