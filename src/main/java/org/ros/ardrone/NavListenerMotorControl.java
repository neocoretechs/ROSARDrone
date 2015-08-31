/**
 * 
 */
package org.ros.ardrone;

import java.util.concurrent.ConcurrentLinkedDeque;

import com.neocoretechs.robocore.MotorControl;

/**
 * @author jg
 *
 */
public class NavListenerMotorControl implements Runnable, NavListenerMotorControlInterface {
	MotorControlInterface2D motorControlListener;
	public static ConcurrentLinkedDeque<NavPacket> data = new ConcurrentLinkedDeque<NavPacket>();
	public static int deleteThreshold = 10; // number of readings before clear
	boolean hasMoved = false;
	/**
	 * Thread spins up on construction
	 */
	public NavListenerMotorControl() {
		motorControlListener = new MotorControl();
		ThreadPoolManager.getInstance().spin(this, "SYSTEM");
	}
	
	/* (non-Javadoc)
	 * @see org.ros.ardrone.NavListenerMotorControlInterface#pushData(org.ros.ardrone.NavPacket)
	 */
	@Override
	public void pushData(NavPacket np) {
		synchronized(data) {
			if(data.size() > deleteThreshold) {
				data.removeLast();
			}
			data.push(np);
			data.notifyAll();
		}
	}
	/* (non-Javadoc)
	 * @see java.lang.Runnable#run()
	 */
	@Override
	public void run() {
		while(true) {
			synchronized(data) {
				if( data.size() == 0 )
					try {
						data.wait();
					} catch (InterruptedException e) {}
				NavPacket np = data.pop();
				int linearMove = 0;
				int courseOffset = 0;
				// we have x,y theta where x is frame x and y is distance to object
				if( np.isVision() ) {
					int range = (int)np.getVisionDistance();
					System.out.println("Range: "+range);
					if( range < 200 ) {
						courseOffset = 0; // stop
						System.out.println("Course offset zero");
					} else {
						int x = (int) np.getVisionX();
						if( x > 500 ) {
							x -=500;
						} else {
							x = 500 - x;
							x = -x; // turn left
						}
						// div by 10 to scale 500 to 0-5 degrees off center turn
						x /= 100;
						courseOffset = x;
						linearMove = 50;
						if( courseOffset != 0 ) linearMove = 0; // turn in place
						System.out.println("Robot offset course with "+ courseOffset +" "+linearMove);
					}
				} else {
					// no vision markers to influence the move
					courseOffset = np.getTargetYaw();
					linearMove = np.getTargetPitch();
					System.out.println("Robot normal course with "+ courseOffset +" "+linearMove);
				}
				hasMoved = motorControlListener.move2DRelative(np.getGyros()[0] , courseOffset, linearMove
						, np.getTimeVal(), np.getAccs(), np.getRanges());
					//System.out.println("Robot should have Moved to "+(robotTheta *= 57.2957795)+" degrees"); // to degrees		
			}
			//try {
			//	System.out.println("NavListenerMotorControl:"+MachineBridge.getInstance("battery").get(0)+" elems:"+data.size());
			//} catch(IndexOutOfBoundsException ioobe) {}
		}
	}
	/* (non-Javadoc)
	 * @see org.ros.ardrone.NavListenerMotorControlInterface#getMotorControlListener()
	 */
	@Override
	public MotorControlInterface2D getMotorControlListener() {
		return motorControlListener;
	}

	/* (non-Javadoc)
	 * @see org.ros.ardrone.NavListenerMotorControlInterface#setMotorControlListener(org.ros.ardrone.MotorControlInterface2D)
	 */
	@Override
	public void setMotorControlListener(MotorControlInterface2D motorControlListener) {
		this.motorControlListener = motorControlListener;
	}
}
