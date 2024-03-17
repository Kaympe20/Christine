// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight extends SubsystemBase {

  public final static double LIMELIGHT_HEIGHT = 0.228; // TODO: Measure and note height in cm
  public final static double LIMELIGHT_ANGLE = 28; // TODO: Measure and note the angle in degrees

  public ShuffleboardTab tab = Shuffleboard.getTab("Targeting");

  public Limelight() {
    setLedMode(0);
    setStreamMode(0);
    tab.addDouble("Distance", this::distance);
    tab.addDouble("Vertical Offset", () -> targetData.verticalOffset);
  }

  public class TargetData {
    public boolean hasTargets = false;
    public double horizontalOffset = 0;
    public double verticalOffset = 0;
    public double targetArea = 0;
    public double skew = 0;
    public double latency = 0;
    public double shortSideLength = 0;
    public double longSideLength = 0;
    public double horizontalSideLength = 0;
    public double verticalSideLength = 0;
  }

  private TargetData targetData = new TargetData();

  public NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault().getTable("Simulating").getStructTopic("EstimatedPose", Pose2d.struct).publish();

  @Override
  public void periodic() {
    SmartDashboard.putString("Stream Mode", (streamMode() == 0) ? "Main" : "Secondary");
    updateTargetData(table);
    
    posePublisher.set(poseEstimation(new Rotation2d(0)));
    
    SmartDashboard.putNumberArray("TargetPose", tagPose());
  }

  private void updateTargetData(NetworkTable table) {
    targetData.hasTargets = table.getEntry("tv").getBoolean(false);
    targetData.horizontalOffset = table.getEntry("tx").getDouble(0.0);
    targetData.verticalOffset = table.getEntry("ty").getDouble(0.0);
    targetData.targetArea = table.getEntry("ta").getDouble(0.0);
    targetData.skew = table.getEntry("ts").getDouble(0.0);
    targetData.latency = table.getEntry("tl").getDouble(0.0);
    targetData.shortSideLength = table.getEntry("tshort").getDouble(0.0);
    targetData.longSideLength = table.getEntry("tlong").getDouble(0.0);
    targetData.horizontalSideLength = table.getEntry("thor").getDouble(0.0);
    targetData.verticalSideLength = table.getEntry("tvert").getDouble(0.0);
  }

  // It's very inaccurate of objects that are same height as the robot
  public double distance() { // NOTE: Maybe make this a bit more general purpose so we can find the distane from an arbitrary point, instead of specifically a Limelight Target
    TargetData targetData = targetData();
    double a2 = targetData.verticalOffset;
    double a1 = LIMELIGHT_ANGLE;
    double h1 = LIMELIGHT_HEIGHT;
    double h2 = 2.05; // CURRENTLY A TEST LOCATION

    double result = h2 - h1;
    double radians = Math.toRadians(a1 + a2);
    double dist = result / Math.tan(radians);

    return Math.abs(dist);
  }

  public Pose2d poseEstimation(Rotation2d rotation) { // cosine is x pose, sine is y pose
    double[] tag = tagPose();

    double rot = (tag[3] == 180) ? -1 : 1;

    double botX = tag[0] + (distance() * Math.acos(Math.toRadians(targetData.horizontalOffset)) * rot);
    double botY = tag[2] - (distance() * Math.asin(Math.toRadians(targetData.horizontalOffset)) * rot);

    return new Pose2d(botX, botY, rotation);
  }



  public TargetData targetData() {
    return targetData;
  }

  public void toggleStreamMode(){
    setStreamMode((streamMode() + 1) % 4);
  }

  public void setCameraMode(int newCameraMode) {
    table.getEntry("camMode").setNumber(newCameraMode);
  }

  public void setLedMode(int newLedMode) {
    table.getEntry("ledMode").setNumber(newLedMode);
  }

  public void setPipeline(int newPipeline) {
    table.getEntry("pipeline").setNumber(newPipeline);
  }

  public void setStreamMode(int newStreamMode) {
    table.getEntry("stream").setNumber(newStreamMode);
  }

  public int ledMode() {
    return (int) table.getEntry("ledMode").getDouble(0.0);
  }

  public int pipeline() {
    return (int) table.getEntry("pipeline").getDouble(0.0);
  }

  public int streamMode() {
    return (int) table.getEntry("stream").getDouble(0.0);
  }

  public double[] botpose() {
    return table.getEntry("stream").getDoubleArray(new double[] {-1,-1,-1, -1,-1,-1});
  }

  public int tagID() {
    return (int) table.getEntry("tid").getInteger(-1);
  }

  public double[] tagPose(){
    int alliance = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) ? 7 : 3;
    switch(alliance){
    
      default:
      return new double[] {-1,-1,-1,-1,};

      
      case 1: // Blue Alliance Player Station
      return new double[]{15.36, 1.22, 0.44, 0};

      case 2: // Blue Alliance Player Station
      return new double[]{15.85, 1.22, 0.85, 0};

      case 3,4: // Red Alliance Speaker
      return new double[]{16.27, 1.32, 5.60, 180};

      case 5: // Red Alliance AMP
      return new double[]{14.64, 1.22, 8.01, 180};

      case 6: // Blue Alliance AMP
      return new double[]{1.89, 1.22, 8.00, 0};

      case 7, 8: // Blue Alliance Speaker
      return new double[]{0.41, 1.32,  5.50, 0};

      case 9: // Red Alliance Player Station
      return new double[]{0.64, 1.22, 0.82, 180};

      case 10: // Red Alliance Playe Station
      return new double[]{1.21, 1.22, 0.50, 180};

      case 11: // Red Alliance Stage
      return new double[]{11.90, 1.21, 3.73, 180};
      
      case 12: // Red Alliance Stage
      return new double[]{11.90, 1.21, 4.53, 180};

      case 13: // Red Alliance Stage
      return new double[]{11.20, 1.21, 4.15, 180};

      case 14: // Blue Alliance Stage
      return new double[]{5.32, 1.21, 4.15, 0};

      case 15: // Blue Alliance Stage
      return new double[]{4.66, 1.21, 4.54, 0};

      case 16: // Blue Alliance Stage
      return new double[]{4.66, 1.21, 3.71, 0};

    }
  }
}