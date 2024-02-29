// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import java.util.HashMap;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public final class Constants {
  public static final double DRIVE_MAX_VELOCITY_METERS_PER_SECOND = 0.5;
  public static final double DRIVE_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 0.2;

  public static final class DriveConstants {
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(25.5);
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(27.5);

    public static final int FRONT_LEFT_DRIVE_MOTOR = 4;
    public static final int FRONT_LEFT_TURN_MOTOR = 5;
    public static final int FRONT_LEFT_ENCODER = 11;

    public static final int FRONT_RIGHT_DRIVE_MOTOR = 2;
    public static final int FRONT_RIGHT_TURN_MOTOR = 3;
    public static final int FRONT_RIGHT_ENCODER = 10;

    public static final int BACK_LEFT_DRIVE_MOTOR = 6;
    public static final int BACK_LEFT_TURN_MOTOR = 7;
    public static final int BACK_LEFT_ENCODER = 12;

    public static final int BACK_RIGHT_DRIVE_MOTOR = 8;
    public static final int BACK_RIGHT_TURN_MOTOR = 9;
    public static final int BACK_RIGHT_ENCODER = 13;

    public static final int PIGEON_ID = 14;

    // public static final double WHEEL_DIAMETER = 0.1016; // Metres
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4); // Metres
    public static final double DRIVE_REDUCTION = (15.0 / 32.0) * (10.0 / 60.0);
    public static final double STEER_REDUCTION = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    public static final double DRIVE_CONVERSION_FACTOR = Math.PI * WHEEL_DIAMETER * DRIVE_REDUCTION;

    public static final int Field_Oriented = 1;
    public static final int Fixed_Point_Tracking = 2;
    public static final int Fixed_Alignment = 3;
    
}
  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 0.2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.2;
    public static final double kMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = 2 * Math.PI;

    public static final double kPXController = 2;
    public static final double kPYController = 2;
    public static final double kPThetaController = 4;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(1,1);
  }

}
