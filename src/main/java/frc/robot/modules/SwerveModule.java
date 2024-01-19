package frc.robot.modules;

public interface SwerveModule {
    public void resetDrivePosition();
    public void resetSteerPosition();
    public double drivePosition();
    public double steerAngle();
    public void resetAbsolute();
    public void stop();
    public void set(double driveVolts, double targetAngle);
}
