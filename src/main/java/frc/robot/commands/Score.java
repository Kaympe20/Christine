package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Flywheel;
import frc.robot.utility.IO;

public class Score extends SequentialCommandGroup{
    public Score(IO io){
        addRequirements(io.shooter, io.intake);
        ProfiledShooter profiledShoot = new ProfiledShooter(io, Flywheel.PASS_OFF_ANGLE);
        addCommands(new ParallelRaceGroup(profiledShoot,
            new SequentialCommandGroup(
                new InstantCommand(() -> io.profiledShoot.setAngle(Flywheel.PASS_OFF_ANGLE)),
                new InstantCommand(() -> profiledShoot.setAngle(Flywheel.PASS_OFF_ANGLE)),
                new InstantCommand(() -> io.shooter.flywheelVoltage(-16)),
                new WaitCommand(0.3),
                new WaitUntilCommand(() -> Math.abs(profiledShoot.controller.getPositionError()) < 3).withTimeout(1.0),
                new InstantCommand(() -> io.shooter.helperVoltage(12)),
                new InstantCommand(() -> io.intake.speed(-1)),
                new WaitCommand(0.2),
                // new InstantCommand(() -> profiledShoot.stop()),
                new InstantCommand(() -> io.shooter.flywheelVoltage(0.0)),
                new InstantCommand(() -> io.shooter.helperVoltage(0.0)),
                new InstantCommand(() -> io.intake.speed(0.0)))));
        
    }
}