/**
 * AR Drone driver for ROS
 * Based on the JavaDrone project and rosjava and ardrone_utd and the original Japanese version and some other
 * assorted code.
 * @author jg
 */
package org.ros.ardrone;

import java.awt.image.BufferedImage;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.message.Time;
import org.jboss.netty.buffer.ChannelBuffers;

import de.yadrone.base.AbstractConfigFactory;
import de.yadrone.base.IARDrone;
import de.yadrone.base.IARDroneLand;
import de.yadrone.base.IDrone;
import de.yadrone.base.navdata.data.Altitude;
import de.yadrone.base.navdata.listener.AltitudeListener;
import de.yadrone.base.navdata.listener.AttitudeListener;
import de.yadrone.base.video.ImageListener;

public class ardrone_land extends AbstractNodeMain  {

	IDrone drone;
	double phi, theta, psi;
	int range;
	byte[] bbuf = new byte[320*240*3];
	boolean started = true;
	boolean videohoriz = true;
	boolean emergency = false;
	double pitch, roll, yaw, vertvel;
	Time tst;
	int imwidth = 320, imheight = 240;
	int[] rgbArray = new int[imwidth*imheight];
	Object vidMutex = new Object(); 
	Object navMutex = new Object();

@Override
public GraphName getDefaultNodeName() {
	return GraphName.of("ardrone");
}


@Override
public void onStart(final ConnectedNode connectedNode) {
	final Log log = connectedNode.getLog();
	Subscriber<geometry_msgs.Twist> subscriber = connectedNode.newSubscriber("cmd_vel", geometry_msgs.Twist._TYPE);
	Subscriber<std_msgs.Empty> substol = connectedNode.newSubscriber("ardrone/activate", std_msgs.Empty._TYPE);
	Subscriber<std_msgs.Empty> subsreset = connectedNode.newSubscriber("ardrone/reset", std_msgs.Empty._TYPE);
	Subscriber<std_msgs.Empty> subschannel = connectedNode.newSubscriber("ardrone/zap", std_msgs.Empty._TYPE);
	final Publisher<geometry_msgs.Quaternion> navpub =
		connectedNode.newPublisher("ardrone/navdata", geometry_msgs.Quaternion._TYPE);
	final Publisher<sensor_msgs.Image> imgpub =
		connectedNode.newPublisher("ardrone/image_raw", sensor_msgs.Image._TYPE);
	final Publisher<sensor_msgs.CameraInfo> caminfopub =
		connectedNode.newPublisher("ardrone/camera_info", sensor_msgs.CameraInfo._TYPE);
	final Publisher<sensor_msgs.Range> rangepub = 
		connectedNode.newPublisher("ardrone/range", sensor_msgs.Range._TYPE);
		
	try{
		drone = (IDrone)AbstractConfigFactory.createFactory("Land").createDrone();
		drone.getNavDataManager().addAttitudeListener(new AttitudeListener() {
			
			public void attitudeUpdated(float pitch, float roll, float yaw)
			{
				synchronized(navMutex) {
					//System.out.println("Pitch: " + pitch + " Roll: " + roll + " Yaw: " + yaw);
					phi = roll;
					theta = pitch;
					//gaz = nd.getAltitude();
					psi = yaw;
				}
			}

			public void attitudeUpdated(float pitch, float roll) { 
				synchronized(navMutex) {
					System.out.println("Pitch: " + pitch + " Roll: " + roll);
				}
			}
			
			public void windCompensation(float pitch, float roll) { 
				synchronized(navMutex) {
					
				}
			}
		});
		
		drone.getNavDataManager().addAltitudeListener(new AltitudeListener() {
			public void receivedExtendedAltitude(Altitude ud) {
				synchronized(navMutex) {
					//System.out.println("Ext. Alt.:"+ud);
					range = ud.getRaw();
				}
			}
			@Override
			public void receivedAltitude(int altitude) {
				synchronized(navMutex) {
					//System.out.println("Altitude: "+altitude);
					if( altitude != 0 ) range = altitude;
				}
			}
		});
		
	    drone.getVideoManager().addImageListener(new ImageListener() {
	            public void imageUpdated(BufferedImage newImage)
	            {
	            	synchronized(vidMutex) {
	            		imwidth = newImage.getWidth();
	            		imheight = newImage.getHeight();
	            		System.out.println("img:"+imwidth+" "+imheight);
	            		//assert( imwidth == 320 && imheight == 240);
	            		//int[] rgbArray = new int[imwidth*imheight];
	            		newImage.getRGB(0, 0, imwidth, imheight, rgbArray, 0, imwidth);
	            		for(int i=0; i < imwidth*imheight; i++)
	            		{
	            			bbuf[i*3 + 2] = (byte)((rgbArray[i] >> 16) & 0xff);
	            			bbuf[i*3 + 1] = (byte)((rgbArray[i] >> 8) & 0xff);
	            			bbuf[i*3] = (byte)(0xff & rgbArray[i]);
	            		}
	            	}
	            }
	    });
		//imwidth = 320;
		//imheight = 240;
	}
	catch(Throwable e)
	{
		e.printStackTrace();
	}


	substol.addMessageListener(new MessageListener<std_msgs.Empty>() {
	@Override
	public void onNewMessage(std_msgs.Empty message) {
		try
		{
			if(started)
			{
				drone.start();
				log.info("The drone is starting.");
			}
			else
			{
				drone.stop();
				log.info("The drone is stopping.");
			}
			started = !started;
		}
		catch (Throwable e)
		{
			e.printStackTrace();
		}
	}
	});
	
	subsreset.addMessageListener(new MessageListener<std_msgs.Empty>() {
	@Override
	public void onNewMessage(std_msgs.Empty message) {
		try
		{
			if(emergency)
			{
				drone.reset();
				log.info("Trying to clear the emergency state.");
			}
			else
			{
				drone.reset();
				log.info("Sending a kill signal to the drone.");
			}
				emergency = !emergency;
			}
		catch (Throwable e)
		{
			e.printStackTrace();
		}
	}
	});

	subschannel.addMessageListener(new MessageListener<std_msgs.Empty>() {
	@Override
	public void onNewMessage(std_msgs.Empty message) {
		try
		{
			synchronized(vidMutex) {
				if(videohoriz) {
					drone.setHorizontalCamera();
					imwidth = 176;
					imheight = 144;
					log.info("Attempting to use the vertical camera.");
				} else {
					drone.setHorizontalCameraWithVertical();
					imwidth = 320;
					imheight = 240;
					log.info("Attempting to use the horizontal camera with vertical.");
				}
				videohoriz = !videohoriz;
			}
		}
		catch (Throwable e)
		{
			e.printStackTrace();
		}
	}
	});
	
	// The Twist message is interpreted just as in Brown's ardrone_brown package.
	subscriber.addMessageListener(new MessageListener<geometry_msgs.Twist>() {
	@Override
	public void onNewMessage(geometry_msgs.Twist message) {
		geometry_msgs.Vector3 val = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
		val = message.getLinear();
		synchronized(navMutex) {
			roll = val.getY();
			pitch = val.getX();
			vertvel = val.getZ();
			val = message.getAngular();
			yaw = val.getZ();
			try
			{
				((IARDroneLand)drone).move2D((int)roll, yaw);
			}
			catch (Throwable e)
			{
				e.printStackTrace();
			}
				log.debug("Drone commands: " + roll + " " + yaw);        
			}
		}
	});

	// This CancellableLoop will be canceled automatically when the node shuts
	// down.
	connectedNode.executeCancellableLoop(new CancellableLoop() {
		private int sequenceNumber;

		@Override
		protected void setup() {
			sequenceNumber = 0;
		}

		@Override
		protected void loop() throws InterruptedException {
			std_msgs.Header imghead = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Header._TYPE);

			geometry_msgs.Quaternion str = navpub.newMessage();
			sensor_msgs.Image imagemess = imgpub.newMessage();
			sensor_msgs.CameraInfo caminfomsg = caminfopub.newMessage();
			sensor_msgs.Range rangemsg = rangepub.newMessage();

			//So far I've been unable to figure out how to fill the K and P matrices
			//using rosjava --J.Pablo
			//double[] K = {imwidth/2.0, 0, imwidth/2.0, 0, 160, 120, 0, 0, 1};
			//double[] P = {160, 0, 160, 0, 0, 160, 120, 0, 0, 0, 1, 0};

			imghead.setSeq(sequenceNumber);
			tst = connectedNode.getCurrentTime();
			imghead.setStamp(tst);
			imghead.setFrameId("0");
			synchronized(vidMutex) {
				imagemess.setData(ChannelBuffers.wrappedBuffer(bbuf));
				imagemess.setEncoding("8UC3");
				imagemess.setWidth(imwidth);
				imagemess.setHeight(imheight);
				imagemess.setStep(imwidth*3);
				imagemess.setIsBigendian((byte)0);
				imagemess.setHeader(imghead);

				caminfomsg.setHeader(imghead);
				caminfomsg.setWidth(imwidth);
				caminfomsg.setHeight(imheight);
				caminfomsg.setDistortionModel("plumb_bob");
				//caminfomsg.setK(K);
				//caminfomsg.setP(P);
			}
			synchronized(navMutex) {
				str.setX(phi);
				str.setY(theta);
				str.setZ(0/*gaz*/);
				str.setW(psi);
				navpub.publish(str);
				imgpub.publish(imagemess);
				caminfopub.publish(caminfomsg);
				rangemsg.setFieldOfView(30);
				rangemsg.setMaxRange(600);
				rangemsg.setMinRange(6);
				rangemsg.setRadiationType(sensor_msgs.Range.ULTRASOUND);
				rangemsg.setRange(range);
				rangepub.publish(rangemsg);
				sequenceNumber++;
				Thread.sleep(50);
			}
		}
	});  


}

}
