package org.ros.ardrone;

public interface NavListenerMotorControlInterface {

	public abstract void pushData(NavPacket np);

	public abstract MotorControlInterface2D getMotorControlListener();

	public abstract void setMotorControlListener(
			MotorControlInterface2D motorControlListener);

}