// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  @Override
  public void periodic() {
    SmartDashboard.putString("Stream Mode", (streamMode() == 0) ? "Main" : "Secondary");

    updateTargetData(table);
    
    
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
    switch(tagID()){
      default:
      return table.getEntry("targetpose_robotspace").getDoubleArray( new double[] {-1,-1,-1, -1,-1,-1}); // TODO: Check if it returns [X,Y,Z,Roll,Pitch,Yaw]
      
      case 4,3: // Red Alliance
      return new double[]{0,2.05,0, 0,0,0}; // TODO: PLACEHOLDER

      // case 7,8: // Blue Alliance
      // return new double[]{0,0,0, 0,0,0}; // TODO: PLACEHOLDER

      // case 5: // Red Alliance
      // return new double[]{0,0,0, 0,0,0}; // TODO: PLACEHOLDER

      // case 6: // Blue Alliance
      // return new double[]{0,0,0, 0,0,0}; // TODO: PLACEHOLDER

    }
  }
}