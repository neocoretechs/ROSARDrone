package org.ros.ardrone;
/**
 * This interface is the cut between ARDrone land/sea/space (basically anything outside of standard air motor control)
 * Preliminary sensor fusion of IMU, ultrasonic range, and accelerometer deltas to determine baseline safe movement
 * @author jg
 *
 */
public interface MotorControlInterface2D {
	public boolean move2DRelative(float yawIMURads, int yawTargetDegrees, int targetDistance, int targetTime, float[] accelDeltas, int[] ranges);
	public boolean move2DAbsolute(float yawIMURads, int yawTargetDegrees, int targetDistance, int targetTime, float[] accelDeltas, int[] ranges);
}
