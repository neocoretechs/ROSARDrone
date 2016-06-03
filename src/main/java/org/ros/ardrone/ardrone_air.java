/// AR Drone driver for ROS
// Based on the JavaDrone project and rosjava.
//
// Author: Juan-Pablo Ramirez
// <pablo.ramirez@utdallas.edu>
// The University of Texas at Dallas
//


package org.ros.ardrone;

import java.awt.image.BufferedImage;
import java.nio.ByteBuffer;
import java.util.concurrent.ConcurrentLinkedDeque;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.message.Time;

import com.twilight.h264.decoder.AVFrame;
import com.twilight.h264.player.FrameUtils;
import com.twilight.h264.player.ImageListener;
import com.twilight.h264.player.RGBListener;

import de.yadrone.base.ARDrone;
import de.yadrone.base.AbstractConfigFactory;
import de.yadrone.base.IARDrone;
import de.yadrone.base.navdata.data.Altitude;
import de.yadrone.base.navdata.listener.AltitudeListener;
import de.yadrone.base.navdata.listener.AttitudeListener;


public class ardrone_air extends AbstractNodeMain  {

	IARDrone drone;
	double phi, theta, psi;
	int range;
	byte[] bbuf;
	boolean landed = true;
	boolean videohoriz = true;
	boolean emergency = false;
	double pitch, roll, yaw, vertvel;
	Time tst;
	int imwidth = 320, imheight = 240;
	Object vidMutex = new Object(); 
	Object navMutex = new Object();
	Object rngMutex = new Object();


@Override
public GraphName getDefaultNodeName() {
	return GraphName.of("pubs_ardrone");
}


@Override
public void onStart(final ConnectedNode connectedNode) {
	final Log log = connectedNode.getLog();
	Subscriber<geometry_msgs.Twist> subscriber = connectedNode.newSubscriber("cmd_vel", geometry_msgs.Twist._TYPE);
	Subscriber<std_msgs.Empty> substol = connectedNode.newSubscriber("ardrone/takeoff", std_msgs.Empty._TYPE);
	Subscriber<std_msgs.Empty> subsreset = connectedNode.newSubscriber("ardrone/reset", std_msgs.Empty._TYPE);
	Subscriber<std_msgs.Empty> subschannel = connectedNode.newSubscriber("ardrone/zap", std_msgs.Empty._TYPE);
	final Publisher<geometry_msgs.Quaternion> navpub =
		connectedNode.newPublisher("ardrone/navdata", geometry_msgs.Quaternion._TYPE);
	final Publisher<sensor_msgs.Image> imgpub =
		connectedNode.newPublisher("ardrone/image_raw", sensor_msgs.Image._TYPE);
	final Publisher<sensor_msgs.CameraInfo> caminfopub =
		connectedNode.newPublisher("ardrone/camera_info", sensor_msgs.CameraInfo._TYPE);
	final Publisher<sensor_msgs.Range> rangepub = 
			connectedNode.newPublisher("range/ultrasonic/ardrone", sensor_msgs.Range._TYPE);		
	try{
		drone = (IARDrone)AbstractConfigFactory.createFactory("Air").createDrone();
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
					//System.out.println("Pitch: " + pitch + " Roll: " + roll);
					phi = roll;
					theta = pitch;
				}
			}
			
			public void windCompensation(float pitch, float roll) { 
				synchronized(navMutex) {
					
				}
			}
		});
		
	    drone.getVideoManager().addImageListener(new RGBListener() {
            public void imageUpdated(AVFrame newImage)
            {
            	synchronized(vidMutex) {
            		imwidth = newImage.imageWidth;
            		imheight = newImage.imageHeight;
					int bufferSize = imwidth * imheight * 3;
					if (bbuf == null || bufferSize != bbuf.length) {
						bbuf = new byte[bufferSize];
					}
					FrameUtils.YUV2RGB(newImage, bbuf); // RGBA32 to BGR8
            	}
            }
	    });
	    
		drone.getNavDataManager().addAltitudeListener(new AltitudeListener() {
			public void receivedExtendedAltitude(Altitude ud) {
				synchronized(rngMutex) {
					//System.out.println("Ext. Alt.:"+ud);
					range = ud.getRaw();
				}
			}
			@Override
			public void receivedAltitude(int altitude) {
				synchronized(rngMutex) {
					//System.out.println("Altitude: "+altitude);
					if( altitude != 0 ) range = altitude;
				}
			}
		});
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
			if(landed)
			{
				drone.takeOff();
				log.info("The drone is taking off.");
			}
			else
			{
				drone.landing();
				log.info("The drone is landing.");
			}
			landed = !landed;
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
				drone.move3D(-(int)roll, -(int)pitch, (int)vertvel, -(int)yaw);
			} catch (Throwable e) {
				e.printStackTrace();
			}
			log.debug("Drone commands: " + roll + " " + pitch + " " + vertvel + " " + yaw);        
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

			imghead.setSeq(sequenceNumber);
			sequenceNumber++;
			tst = connectedNode.getCurrentTime();
			imghead.setStamp(tst);
			imghead.setFrameId("0");
			synchronized(vidMutex) {
				if( bbuf != null ) {
					imagemess.setData(ByteBuffer.wrap(bbuf));
					imagemess.setEncoding("8UC3");
					imagemess.setWidth(imwidth);
					imagemess.setHeight(imheight);
					imagemess.setStep(imwidth*3);
					imagemess.setIsBigendian((byte)0);
					imagemess.setHeader(imghead);
					imgpub.publish(imagemess);
				
					caminfomsg.setHeader(imghead);
					caminfomsg.setWidth(imwidth);
					caminfomsg.setHeight(imheight);
					caminfomsg.setDistortionModel("plumb_bob");
					//caminfomsg.setK(K);
					//caminfomsg.setP(P);
					caminfopub.publish(caminfomsg);
				}
			}
			Thread.sleep(1);
			synchronized(navMutex) {
				str.setX(phi);
				str.setY(theta);
				str.setZ(range);
				str.setW(psi);
				navpub.publish(str);
			}
			Thread.sleep(1);
			synchronized(rngMutex) {
				rangemsg.setFieldOfView(35);
				rangemsg.setMaxRange(6000);
				rangemsg.setMinRange(0);
				rangemsg.setRadiationType(sensor_msgs.Range.ULTRASOUND);
				rangemsg.setRange(range);
				rangepub.publish(rangemsg);
			}
			Thread.sleep(1);
		}
	});  


}

}
