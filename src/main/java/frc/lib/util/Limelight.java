package frc.lib.util;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class Limelight {
    private final double limelightHeight;
    private final Rotation2d limelightAngle;
    private final double targetHeight;
    private MedianFilter xMedian;
    private MedianFilter yMedian;

    /**
     * @param limelightHeight Height (in meters) to center of limelight camera from the ground.
     * @param limelightAngle Angle of limelight from horizontal (0 degrees is straight forward, 90 is straight up).
     * @param targetHeight Height (in meters) to target from the ground.
     */
    public Limelight(double limelightHeight, Rotation2d limelightAngle, double targetHeight) {
        this.limelightHeight = limelightHeight;
        this.limelightAngle = limelightAngle;
        this.targetHeight = targetHeight;
        // distanceMedian = new MedianFilter(10);
        xMedian = new MedianFilter(10);
        yMedian = new MedianFilter(10);
    }

    /**@return Horizontal Offset From Crosshair To Target in degrees
     * (Note: Inverted from standard LL provided angle to be CCW+) */
    public Rotation2d getTx() {
        return Rotation2d.fromDegrees(xMedian.calculate(-NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0)));
    }

    /**@return Vertical Offset from Crosshair to Target in degrees */
    public Rotation2d getTy() {
        return Rotation2d.fromDegrees(yMedian.calculate(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0)));
    }

    /**
     * Checks for Target, and also resets distanceFilter if no target
     * @return True if LL detects Target 
     */
    public boolean hasTarget(){
        boolean hasTarget = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1;
        if (!hasTarget){
            //distanceMedian.reset();
            resetFilters();
        }
        return hasTarget;
    }

    // /**@return Median filtered distance to Target in meters */
    // public Translation2d getDistance(){
    //     double heightDifference = targetHeight - limelightHeight;
    //     Rotation2d combinedAngle = limelightAngle.plus(getTy());
    //     return new Translation2d(heightDifference / combinedAngle.getTan(), 0);//distanceMedian.calculate((heightDifference / combinedAngle.getTan())), 0);
    // }

    /**@return Median filtered distance to Target in meters */
    public Translation2d getDistance(){
        double heightDifference = targetHeight - limelightHeight;
        Rotation2d combinedAngle = limelightAngle.plus(getTy());
        double distance = (heightDifference / combinedAngle.getTan()) + (Constants.Vision.goalDiameter / 2);
        return new Translation2d(distance, 0);
    }

    /**
     * Needs to be called whenever the Limelight
     */
    public void resetFilters(){
        // distanceMedian.reset();
        xMedian.reset();
        yMedian.reset();
    }

    /**@return The pipeline’s latency contribution in seconds  */
    public double getLatency() {
        double ntLatency = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tl").getDouble(0);
        return ((ntLatency + 11) / 1000); //11ms Capture Latency
    }

    /**
     * Use to set state of LL's leds
     */
    public void ledState(ledStates ledState){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(ledState.value);
    }

    /** Use to set LL camera mode
     * @param camMode
     * <pre>
     * 0: driver
     * 1: vision
     */
    public void camMode(int camMode){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(camMode);
    }

    /** Use to set active pipeline
     * @param pipeline 0-9
     */
    public void setPipeline(int pipeline){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
    }
    
    /** Possible values for Led States. */
    public enum ledStates {
        pipeline(0),
        off(1),
        blink(2),
        on(3);
        
        public final int value;
        ledStates(int value) {
          this.value = value;
        }
    }
}
