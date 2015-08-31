package org.ros.ardrone;

public class NavPacket {
	float pitch = 0.0f;
	float roll = 0.0f;
	int targetYaw = 0;
	int targetPitch = 0;
	int timeVal;
	int[] ranges;
	float[] accs = new float[3]; // accelerometer values
	float gyros[] = new float[3]; // gyro
	boolean isVision;
	int visionX;
	int visionDistance;
	
	/**
	 * 
	 * @param tgyros
	 * @param taccs
	 * @param tranges
	 * @param pitch
	 * @param roll
	 * @param targetYaw in degrees, integer
	 * @param targetPitch linear distance forward travel or arc radius mm
	 * @param i
	 * @param visionDistance 
	 * @param visionX 
	 * @param isVision 
	 */
	public NavPacket(float[] tgyros, float[] taccs, int[] tranges, float pitch, float roll, 
			int targetYaw, int targetPitch, int i, 
			boolean isVision, int visionX, int visionDistance) {
		this.pitch = pitch;
		this.roll = roll;
		this.targetYaw = targetYaw;
		this.targetPitch = targetPitch;
		timeVal = i;
		accs = taccs;
		gyros = tgyros;
		ranges = tranges;
		this.isVision = isVision;
		this.visionX = visionX;
		this.visionDistance = visionDistance;
	}
	public boolean isVision() {
		return isVision;
	}
	public int getVisionX() {
		return visionX;
	}
	public void setVisionX(int visionX) {
		this.visionX = visionX;
	}
	public int getVisionDistance() {
		return visionDistance;
	}
	public void setVisionDistance(int visionDistance) {
		this.visionDistance = visionDistance;
	}
	public float getPitch() {
		return pitch;
	}
	public void setPitch(float pitch) {
		this.pitch = pitch;
	}
	public float getRoll() {
		return roll;
	}
	public void setRoll(float roll) {
		this.roll = roll;
	}
	public int getTargetYaw() {
		return targetYaw;
	}
	/**
	 * in degrees
	 * @param targetYaw
	 */
	public void setTargetYaw(int targetYaw) {
		this.targetYaw = targetYaw;
	}
	public int getTargetPitch() {
		return targetPitch;
	}
	/**
	 * linear distance in mm
	 * @param targetPitch
	 */
	public void setTargetPitch(int targetPitch) {
		this.targetPitch = targetPitch;
	}
	public int getTimeVal() {
		return timeVal;
	}
	public void setTimeVal(int timeVal) {
		this.timeVal = timeVal;
	}
	public int[] getRanges() {
		return ranges;
	}
	public void setRanges(int[] ranges) {
		this.ranges = ranges;
	}
	public float[] getAccs() {
		return accs;
	}
	public void setAccs(float[] accs) {
		this.accs = accs;
	}
	public float[] getGyros() {
		return gyros;
	}
	public void setGyros(float[] gyros) {
		this.gyros = gyros;
	}

	
}
