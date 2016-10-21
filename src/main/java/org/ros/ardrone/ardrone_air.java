/// AR Drone driver for ROS
// Based on the JavaDrone project and rosjava.
//
// Author: Juan-Pablo Ramirez
// <pablo.ramirez@utdallas.edu>
// The University of Texas at Dallas
//


package org.ros.ardrone;

import geometry_msgs.Quaternion;

import java.awt.image.BufferedImage;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
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

import std_msgs.Int16;

import com.sun.image.codec.jpeg.JPEGCodec;
import com.sun.image.codec.jpeg.JPEGImageEncoder;
import com.twilight.h264.decoder.AVFrame;
import com.twilight.h264.player.FrameUtils;
import com.twilight.h264.player.ImageListener;
import com.twilight.h264.player.RGBListener;

import de.yadrone.base.ARDrone;
import de.yadrone.base.AbstractConfigFactory;
import de.yadrone.base.IARDrone;
import de.yadrone.base.IDrone;
import de.yadrone.base.navdata.accel.AcceleroPhysData;
import de.yadrone.base.navdata.accel.AcceleroRawData;
import de.yadrone.base.navdata.data.Altitude;
import de.yadrone.base.navdata.data.GyroPhysData;
import de.yadrone.base.navdata.data.GyroRawData;
import de.yadrone.base.navdata.data.KalmanPressureData;
import de.yadrone.base.navdata.data.MagnetoData;
import de.yadrone.base.navdata.data.Pressure;
import de.yadrone.base.navdata.data.Temperature;
import de.yadrone.base.navdata.data.TrackerData;
import de.yadrone.base.navdata.listener.AcceleroListener;
import de.yadrone.base.navdata.listener.AltitudeListener;
import de.yadrone.base.navdata.listener.AttitudeListener;
import de.yadrone.base.navdata.listener.GyroListener;
import de.yadrone.base.navdata.listener.MagnetoListener;
import de.yadrone.base.navdata.listener.PressureListener;
import de.yadrone.base.navdata.listener.TemperatureListener;
import de.yadrone.base.navdata.listener.VisionListener;
import de.yadrone.base.navdata.vision.VisionData;
import de.yadrone.base.navdata.vision.VisionPerformance;
import de.yadrone.base.navdata.vision.VisionTag;


public class ardrone_air extends ardrone_land  {
	private static boolean DEBUG = false;

	boolean landed = true;
	boolean videohoriz = true;
	boolean emergency = false;
	double pitch, roll, yaw, vertvel;
	Time tst;


@Override
public GraphName getDefaultNodeName() {
	return GraphName.of("ardrone2");
}
/**
 * 
 * Start the main processing pipeline. We subscribe to an external ranging message robocore/range to augment the
 * ultrasonic range and supply a complete point cloud to the range/ultrasonic/ardrone channel. If we have ultrasonics
 * they will be in the first elements of the point cloud array in X in addition to element 0 X being the ARDrone ranger
 */
@Override
public void onStart(final ConnectedNode connectedNode) {

	final Log log = connectedNode.getLog();
	//Subscriber<geometry_msgs.Twist> subsmotion = connectedNode.newSubscriber("cmd_vel", geometry_msgs.Twist._TYPE);
	Subscriber<geometry_msgs.Twist> subscriber = connectedNode.newSubscriber("cmd_vel", geometry_msgs.Twist._TYPE);
	Subscriber<std_msgs.Empty> substof = connectedNode.newSubscriber("ardrone/takeoff", std_msgs.Empty._TYPE);
	Subscriber<std_msgs.Empty> subslan = connectedNode.newSubscriber("ardrone/land", std_msgs.Empty._TYPE);
	Subscriber<std_msgs.Empty> subsfor = connectedNode.newSubscriber("ardrone/forward", std_msgs.Empty._TYPE);
	Subscriber<std_msgs.Empty> subsbak = connectedNode.newSubscriber("ardrone/backward", std_msgs.Empty._TYPE);
	Subscriber<std_msgs.Empty> subsup = connectedNode.newSubscriber("ardrone/up", std_msgs.Empty._TYPE);
	Subscriber<std_msgs.Empty> subsdown = connectedNode.newSubscriber("ardrone/down", std_msgs.Empty._TYPE);
	Subscriber<std_msgs.Empty> subsgol = connectedNode.newSubscriber("ardrone/goleft", std_msgs.Empty._TYPE);
	Subscriber<std_msgs.Empty> subsgor = connectedNode.newSubscriber("ardrone/goright", std_msgs.Empty._TYPE);
	Subscriber<std_msgs.Empty> subsspinl = connectedNode.newSubscriber("ardrone/spinleft", std_msgs.Empty._TYPE);
	Subscriber<std_msgs.Empty> subsspinr = connectedNode.newSubscriber("ardrone/spinright", std_msgs.Empty._TYPE);
	Subscriber<std_msgs.Int16> subsspeed = connectedNode.newSubscriber("ardrone/setspeed", std_msgs.Int16._TYPE);
	Subscriber<std_msgs.Int16> subsalt = connectedNode.newSubscriber("ardrone/setalt", std_msgs.Int16._TYPE);
	Subscriber<std_msgs.Empty> subsfreeze = connectedNode.newSubscriber("ardrone/freeze", std_msgs.Empty._TYPE);
	Subscriber<std_msgs.Empty> subshov = connectedNode.newSubscriber("ardrone/hover", std_msgs.Empty._TYPE);
	//final Publisher<sensor_msgs.Image> imgpub =
	//	connectedNode.newPublisher("ardrone/image_raw", sensor_msgs.Image._TYPE);
	//final Publisher<sensor_msgs.CameraInfo> caminfopub =
	//	connectedNode.newPublisher("ardrone/camera_info", sensor_msgs.CameraInfo._TYPE);
	
	Subscriber<std_msgs.Empty> subsreset = connectedNode.newSubscriber("ardrone/reset", std_msgs.Empty._TYPE);
	// Emergency stop message
	Subscriber<std_msgs.Empty> subschannel = connectedNode.newSubscriber("ardrone/zap", std_msgs.Empty._TYPE);

	// navpub pushes navigation data from ARDrone IMU
	final Publisher<sensor_msgs.Imu> navpub =
	connectedNode.newPublisher("ardrone/navdata", sensor_msgs.Imu._TYPE);
	// imgpub has raw camera image from ARDrone selected camera
	final Publisher<sensor_msgs.Image> imgpub =
	connectedNode.newPublisher("ardrone/image_raw", sensor_msgs.Image._TYPE);
	// caminfopub has camera info
	//final Publisher<sensor_msgs.CameraInfo> caminfopub =
	//connectedNode.newPublisher("ardrone/camera_info", sensor_msgs.CameraInfo._TYPE);
	// rangepub has point cloud data of ultrasonic and other range finders all rolled into one, 
	// assume first 2 points upper and lower ultrasonics
	final Publisher<sensor_msgs.Range> rangepub = 
			connectedNode.newPublisher("range/ultrasonic/ardrone", sensor_msgs.Range._TYPE);
	// statpub has status alerts that may come from ARDrone extreme attitude, temp etc.
	//final Publisher<diagnostic_msgs.DiagnosticStatus> statpub =
	//		connectedNode.newPublisher("robocore/status", diagnostic_msgs.DiagnosticStatus._TYPE);
	final Publisher<sensor_msgs.Temperature> temppub = 
		connectedNode.newPublisher("ardrone/temperature", sensor_msgs.Temperature._TYPE);
	final Publisher<sensor_msgs.FluidPressure> presspub = 
		connectedNode.newPublisher("ardrone/pressure", sensor_msgs.FluidPressure._TYPE);
	final Publisher<sensor_msgs.MagneticField> magpub = 
		connectedNode.newPublisher("ardrone/magnetic_field", sensor_msgs.MagneticField._TYPE);
	// Image recognition from drone. Position and orientation of detected image in frame
	final Publisher<geometry_msgs.Quaternion> tagpub = 
		connectedNode.newPublisher("ardrone/image_tag", geometry_msgs.Quaternion._TYPE);
	//final Publisher<diagnostic_msgs.DiagnosticStatus> statpub =
	//		connectedNode.newPublisher("robocore/status", diagnostic_msgs.DiagnosticStatus._TYPE);

	//final Map<String, String> environment;



	try {
		drone = (IARDrone)AbstractConfigFactory.createFactory("Air").createDrone();
		drone.getNavDataManager().addAttitudeListener(new AttitudeListener() {
		// theta phi psi
		public void attitudeUpdated(float tpitch, float troll, float tyaw)
		{
			synchronized(navMutex) {
				//System.out.println("Pitch: " + pitch + " Roll: " + roll + " Yaw: " + yaw);
				phi = troll;
				theta = tpitch;
				//gaz = nd.getAltitude();
				psi = tyaw;
				//pitch = tpitch;
				//roll = troll;
				//yaw = tyaw;
			}
		}
		// these are the euler angles vs raw data above
		public void attitudeUpdated(float tpitch, float troll) { 
			synchronized(navMutex) {
				phi = troll;
				theta = tpitch;
				//System.out.println("Pitch: " + pitch + " Roll: " + roll);
				//((IARDroneLand)drone).move2D((int)phi,(float) theta);
				//pitch = tpitch;
				//roll = troll;
			}
		}
		
		public void windCompensation(float pitch, float roll) { 
			synchronized(navMutex) {
				
			}
		}
	});
	/**
	 * Upper ARDrone ultrasonic ranger
	 */
	drone.getNavDataManager().addAltitudeListener(new AltitudeListener() {
		public void receivedExtendedAltitude(Altitude ud) {
			synchronized(rngMutex) {
				//System.out.println("Ext. Alt.:"+ud);
				if( ud.getRaw() != 0 ) {
					rangeTop = ud.getRaw();
				}
			}
		}
		@Override
		public void receivedAltitude(int altitude) {
			synchronized(rngMutex) {
				//System.out.println("Altitude: "+altitude);
				if( altitude != 0 ) {
					rangeTop = altitude;
				}
			}
		}
	});
	
	final ByteArrayOutputStream os = new ByteArrayOutputStream();
	final JPEGImageEncoder encoder = JPEGCodec.createJPEGEncoder(os);
	
    drone.getVideoManager().addImageListener(new RGBListener() {
        public void imageUpdated(AVFrame newImage)
        {
        	//int bufferSize;
        	if( imageReady ) return; // created but not yet published
			//if (bbuf == null || bufferSize != bbuf.capacity()) {
			//		bbuf = ByteBuffer.allocate(bufferSize);
			//}
        	synchronized(vidMutex) {
			if( bbuf == null ) {
            		imwidth = newImage.imageWidth;
            		imheight = newImage.imageHeight;
            		//bufferSize = imwidth * imheight * 3;	
            		//bbuf = new byte[bufferSize];
			}
			BufferedImage bi = FrameUtils.imageFromFrame(newImage);//.YUV2RGB(newImage, bbuf);
			try {
				//ByteArrayOutputStream os = new ByteArrayOutputStream();
				//JPEGImageEncoder encoder = JPEGCodec.createJPEGEncoder(os);
				encoder.encode(bi);
				os.flush();
				bbuf =  os.toByteArray();
				os.reset();
				//os.close();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}//
			//try {
				//vidbuf.add(bbuf);
				if( DEBUG )
					System.out.println("Added frame "+imwidth+","+imheight+" "+bbuf.length);
			//} catch(IllegalStateException ise) {
				// buffer full;
				//System.out.println("Video buffer full!");
				//vidbuf.clear();
			//}
			
        	} // vid mutex
        	imageReady = true;
        }
    });

	/*
	drone.getNavDataManager().addBatteryListener(new NavListenerMotorControl() {	
		public void batteryLevelChanged(int percentage)
		{
			System.out.println("Battery: " + percentage + " %");
		}
		public void voltageChanged(int vbat_raw) { }
	});
	*/
	
	drone.getNavDataManager().addGyroListener(new GyroListener() {
		@Override
		public void receivedRawData(GyroRawData d) {
			//System.out.println("GyroRaw:"+d);	
		}
		@Override
		public void receivedPhysData(GyroPhysData d) {
			synchronized(gyros) {
				gyros = d.getPhysGyros();
			}
			//System.out.println("GyroPhys:"+d);	
		}
		@Override
		public void receivedOffsets(float[] offset_g) {
			//System.out.print("GyroOffs:");
			//for(float f: offset_g) System.out.print("offs:"+f+" ");
			//System.out.println();
		}

	});
	
    
	drone.getNavDataManager().addAcceleroListener(new AcceleroListener() {
		@Override
		public void receivedRawData(AcceleroRawData d) {
			//System.out.println("Raw Accelero:"+d);
		}

		@Override
		public void receivedPhysData(AcceleroPhysData d) {
			synchronized(accs) {
				accs = d.getPhysAccs();
			}
			//System.out.println("Phys Accelero:"+d);
		}

	});
	
	drone.getNavDataManager().addMagnetoListener(new MagnetoListener() {
		@Override
		public void received(MagnetoData d) {
			//System.out.println("Mag:"+d);
			synchronized(magMutex) {
				mag = d.getM();
			}
		}

	});
	
	drone.getNavDataManager().addPressureListener(new PressureListener() {
		@Override
		public void receivedKalmanPressure(KalmanPressureData d) {
			//System.out.println("kalman Pressure:"+d);
		}
		@Override
		public void receivedPressure(Pressure d) {
			if( d.getMeasurement() != pressure ) {
				pressure = d.getMeasurement();
				isPress = true;
			}
			//System.out.println("Pressure:"+d);

		}

	});
	
	drone.getNavDataManager().addTemperatureListener(new TemperatureListener() {
		@Override
		public void receivedTemperature(Temperature d) {
			//System.out.println("Temp:"+d);
			if( d.getValue() != temperature ) {
				temperature = d.getValue();
				isTemp = true;
			}
		}
	});
	/**
	 * Machine vision is enabled. When messages come where we must multiples them
	 * tagsDetected get array of VisionTag. getSource() returns a DetectionType where values
	 * cmdsuf is 'detections_select_h' dtname is 'HORIZONTAL'
	 */
	drone.getNavDataManager().addVisionListener( new VisionListener() {
		@Override
		public void tagsDetected(VisionTag[] tags) {
			synchronized(visMutex) {
			for(VisionTag vt : tags ) {
				visionDistance = vt.getDistance();
				visionAngle = (int) vt.getOrientationAngle();
				// 0,0 top left 1000,1000 right bottom regardless of camera
				visionX = vt.getX();
				visionY = vt.getY();
				visionName = vt.getSource().name();
				//DetectionType dt = vt.getSource();
				//String cmdsuf = dt.getCmdSuffix();
				//String dtname = dt.name();
				// CadType.H_ORIENTED_COCARDE 
				// The following lines fail to work
				// int vta = vt.getType();
				// if( VisionTagType.fromInt(vta) == VisionTagType.ORIENTED_ROUNDEL)
				//	System.out.println("My toy!");
				//if( visionDistance < 500 ) {
				//System.out.println("Marker "+vt);
				isVision = true;
				break;
				//}
			}
			}	
		}
		
		@Override
		public void trackersSend(TrackerData trackersData) {
			//System.out.println("TrackerData:"+trackersData);
			/*
			int[][][] d3 = trackersData.getTrackers();
			for( int i = 0; i < d3.length; i++) {
				for(int j = 0; j < d3[i].length; j++) {
					System.out.println(d3[i][j][0]+" "+d3[i][j][1]+" "+d3[i][j][2]);
				}					
			}
			*/	
		}
		
		@Override
		public void receivedPerformanceData(VisionPerformance d) {
			//System.out.println("Perf data:"+d);
			
		}
		@Override
		public void receivedRawData(float[] vision_raw) {
			//System.out.println("Vision Raw:"+vision_raw);
			//for(float vis: vision_raw)
			//	System.out.println("raw vis:"+vis);
		}
		@Override
		public void receivedData(VisionData d) {
			//int alt = d.getAltitudeCapture();
			//System.out.println("Vision data range:"+alt+" angles:"+d.getPhiCapture()+" t:"+d.getThetaCapture()+" psi:"+d.getPsiCapture());
			//float[] body = d.getBodyV();
			//for(float bod : body) {
			//	System.out.println("Body V:"+bod);
			//}
		}
		
		@Override
		public void receivedVisionOf(float[] of_dx, float[] of_dy) {
			//System.out.print("VisionOf:");
			//for(float of: of_dx) System.out.println(of);
			//for(float of: of_dy) System.out.println(of);
		}
		@Override
		public void typeDetected(int detection_camera_type) {
			//System.out.println("Cam:"+detection_camera_type);	
		}
		
	});


} catch(Throwable e) {
	e.printStackTrace();
}
	substof.addMessageListener(new MessageListener<std_msgs.Empty>() {
	@Override
	public void onNewMessage(std_msgs.Empty message) {
		try {
			((ARDrone)drone).takeOff();
			if( DEBUG ) log.info("The drone is taking off.");
			landed = false;
		} catch (Throwable e) {
			e.printStackTrace();
		}
	}
	});
	
	subslan.addMessageListener(new MessageListener<std_msgs.Empty>() {
	@Override
	public void onNewMessage(std_msgs.Empty message) {
		try {
				((ARDrone)drone).landing();
				if( DEBUG ) log.info("The drone is landing.");
			landed = true;
		} catch (Throwable e) {
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
				((ARDrone) drone).move3D(-(int)roll, -(int)pitch, (int)vertvel, -(int)yaw);
			} catch (Throwable e) {
				e.printStackTrace();
			}
			log.debug("Drone commands: " + roll + " " + pitch + " " + vertvel + " " + yaw);        
		}
	}
	});

subsfor.addMessageListener(new MessageListener<std_msgs.Empty>() {
@Override
public void onNewMessage(std_msgs.Empty message) {
	try
	{
		drone.forward();
		if( DEBUG )log.info("Forward.");
	} catch (Throwable e) {
		e.printStackTrace();
	}
}
});

subsbak.addMessageListener(new MessageListener<std_msgs.Empty>() {
@Override
public void onNewMessage(std_msgs.Empty message) {
	try
	{
		drone.backward();
		if( DEBUG )log.info("Backward.");
	} catch (Throwable e) {
		e.printStackTrace();
	}
}
});

subsup.addMessageListener(new MessageListener<std_msgs.Empty>() {
@Override
public void onNewMessage(std_msgs.Empty message) {
	try
	{
		((IARDrone)drone).up();
		if( DEBUG )log.info("Up.");
	} catch (Throwable e) {
		e.printStackTrace();
	}
}
});

subsdown.addMessageListener(new MessageListener<std_msgs.Empty>() {
@Override
public void onNewMessage(std_msgs.Empty message) {
	try
	{
		((IARDrone)drone).down();
		if( DEBUG )log.info("Down.");
	} catch (Throwable e) {
		e.printStackTrace();
	}
}
});

subsgol.addMessageListener(new MessageListener<std_msgs.Empty>() {
@Override
public void onNewMessage(std_msgs.Empty message) {
	try
	{
		((IARDrone)drone).goLeft();
		if( DEBUG )log.info("Left.");
	} catch (Throwable e) {
		e.printStackTrace();
	}
}
});

subsgor.addMessageListener(new MessageListener<std_msgs.Empty>() {
@Override
public void onNewMessage(std_msgs.Empty message) {
	try
	{
		((IARDrone)drone).goRight();
		if( DEBUG )log.info("Right.");
	} catch (Throwable e) {
		e.printStackTrace();
	}
}
});

subsspinl.addMessageListener(new MessageListener<std_msgs.Empty>() {
@Override
public void onNewMessage(std_msgs.Empty message) {
	try
	{
		((IARDrone)drone).spinLeft();
		if( DEBUG )log.info("Spin Left.");
	} catch (Throwable e) {
		e.printStackTrace();
	}
}
});

subsspinr.addMessageListener(new MessageListener<std_msgs.Empty>() {
@Override
public void onNewMessage(std_msgs.Empty message) {
	try
	{
		((IARDrone)drone).spinRight();
		if( DEBUG )log.info("Spin Right.");
	} catch (Throwable e) {
		e.printStackTrace();
	}
}
});

/**
 * Set speed as a percentage
 */
subsspeed.addMessageListener(new MessageListener<std_msgs.Int16>() {
@Override
public void onNewMessage(std_msgs.Int16 message) {
	try
	{
		drone.setSpeed(message.getData());
		if( DEBUG )log.info("Speed.");
	} catch (Throwable e) {
		e.printStackTrace();
	}
}
});

/**
 * Freeze.
 */
subsfreeze.addMessageListener(new MessageListener<std_msgs.Empty>() {
@Override
public void onNewMessage(std_msgs.Empty message) {
	try
	{
		drone.freeze();
		if( DEBUG )log.info("Freeze.");
	} catch (Throwable e) {
		e.printStackTrace();
	}
}
});

subshov.addMessageListener(new MessageListener<std_msgs.Empty>() {
@Override
public void onNewMessage(std_msgs.Empty message) {
	try
	{
		((IARDrone)drone).hover();
		if( DEBUG )log.info("Freeze.");
	} catch (Throwable e) {
		e.printStackTrace();
	}
}
});
/**
 * Set max altitude
 */
subsalt.addMessageListener(new MessageListener<std_msgs.Int16>() {
@Override
public void onNewMessage(Int16 message) {
	try
	{
			((IARDrone)drone).setMaxAltitude(message.getData());
			if( DEBUG )log.info("Speed.");
	} catch (Throwable e) {
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
			log.info("Sending a kill signal to the robot.");
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
		// image width and height set later, these are just setups
		synchronized(vidMutex) {
			if(videohoriz) {
				drone.setHorizontalCamera();
				//imwidth = 672;
				//imheight = 418;
				log.info("Attempting to use the vertical camera.");
			} else {
				drone.setHorizontalCameraWithVertical();
				//imwidth = 672;
				//imheight = 418;
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


/**
 * Main publishing loop. Essentially we are publishing the data in whatever state its in, using the
 * mutex appropriate to establish critical sections. A sleep follows each publication to keep the bus arbitrated
 * This CancellableLoop will be canceled automatically when the node shuts down
 */
connectedNode.executeCancellableLoop(new CancellableLoop() {
	private int sequenceNumber;

	@Override
	protected void setup() {
		sequenceNumber = 0;
	}

	@Override
	protected void loop() throws InterruptedException {
		std_msgs.Header imghead = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Header._TYPE);

		imghead.setSeq(sequenceNumber);
		tst = connectedNode.getCurrentTime();
		imghead.setStamp(tst);
		imghead.setFrameId(tst.toString());
		sensor_msgs.Image imagemess = imgpub.newMessage();
		//if( bbuf != null ) {
		//int[] bbuf = vidbuf.poll();
	
		if( bbuf != null && imageReady) {
			synchronized(vidMutex) {		
			//sensor_msgs.CameraInfo caminfomsg = caminfopub.newMessage();
        	//System.out.println("Image:"+newImage.imageWidth+","+newImage.imageHeight+" queue:"+list.size());
				imagemess.setData(ByteBuffer.wrap(bbuf));
			}
			imagemess.setEncoding("JPG"/*"8UC3"*/);
			
			imagemess.setWidth(imwidth);
			imagemess.setHeight(imheight);
			imagemess.setStep(imwidth);
			imagemess.setIsBigendian((byte)0);
			imagemess.setHeader(imghead);
			//
			//caminfomsg.setHeader(imghead);
			//caminfomsg.setWidth(imwidth);
			//caminfomsg.setHeight(imheight);
			//caminfomsg.setDistortionModel("plumb_bob");
			//caminfomsg.setK(K);
			//caminfomsg.setP(P);
			
			imgpub.publish(imagemess);
			imageReady = false;
			if( DEBUG )
				System.out.println("Pub. Image:"+sequenceNumber);
			//caminfopub.publish(caminfomsg);
			//System.out.println("Pub cam:"+imagemess);
			sequenceNumber++;  	
		}

		
		Thread.sleep(1);
		
		//
		// Begin IMU message construction for eventual outbound queueing
		//
		sensor_msgs.Imu str = navpub.newMessage();
		
		// get accelerometer data and populate linear acceleration field
		geometry_msgs.Vector3 val = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE); 
		synchronized(accs) {
				val.setX(accs[0]);
				val.setY(accs[1]);
				val.setZ(accs[2]);
		}
		str.setLinearAcceleration(val);
		
		// get gyro data and populate angular velocity fields
		geometry_msgs.Vector3 valg = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE); 
		synchronized(gyros) {
				valg.setX(gyros[0]);
				valg.setY(gyros[1]);
				valg.setZ(gyros[2]);
		}
		str.setAngularVelocity(valg);
			
		geometry_msgs.Quaternion valq = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Quaternion._TYPE);
		synchronized(navMutex) {
			valq.setX(phi); // roll
			valq.setY(theta); // pitch
			valq.setZ(0.0); // gaz
			valq.setW(psi); //yaw
		}
		str.setOrientation(valq);
		// Publish IMU data to ardrone/navdata
		navpub.publish(str);
		if( DEBUG )
			printGyro(str.getLinearAcceleration(), str.getAngularVelocity(), str.getOrientation());
		//
		//if(DEBUG)
		//	System.out.println("Pub nav:"+str);
		Thread.sleep(1);
		
		// Generate point cloud data which will include
		// ultrasonic range info and any other ranging we field
		sensor_msgs.Range rangemsg = rangepub.newMessage();
		//rangemsg.setFieldOfView(35);
		//rangemsg.setMaxRange(6000);
		//rangemsg.setMinRange(0);
		//rangemsg.setRadiationType(sensor_msgs.Range.ULTRASOUND);
		synchronized(rngMutex) {
				rangemsg.setRange(rangeTop);
		}
		// Publish data to range/ultrasonic/ardrone
		rangepub.publish(rangemsg);
		//if(DEBUG)
		//	System.out.println("Pub rangeTop:"+rangemsg);
		
		if( isTemp ) {
			sensor_msgs.Temperature tmpmsg = temppub.newMessage();
			tmpmsg.setTemperature(temperature);
			temppub.publish(tmpmsg);
			isTemp = false;
		}
		
		if( isPress ) {
			sensor_msgs.FluidPressure pressmsg = presspub.newMessage();
			pressmsg.setFluidPressure(pressure);
			presspub.publish(pressmsg);
			isPress = false;
		}
		
		sensor_msgs.MagneticField magmsg = magpub.newMessage();
		geometry_msgs.Vector3 magv = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
		synchronized(magMutex) {
			magv.setX(mag[0]);
			magv.setY(mag[1]);
			magv.setZ(mag[2]);
			magmsg.setMagneticField(magv);
		}
		magpub.publish(magmsg);
		
		// ARDrone vision recognition tags, if detected
		// Publishes to bus
		if( isVision) {
			/*DiagnosticStatus*/Quaternion tagmsg = tagpub.newMessage();
			synchronized(visMutex) {
				/*
				ArrayList<KeyValue> orient = new ArrayList<KeyValue>();
				KeyValue vx = new KeyValue();
				vx.setKey("X");
				vx.setValue(String.valueOf(visionX));
				KeyValue vy = new KeyValue();
				vy.setKey("Y");
				vy.setValue(String.valueOf(visionY));
				KeyValue vz = new KeyValue();
				vz.setKey("Z Distance");
				vz.setValue(String.valueOf(visionDistance));
				KeyValue vw = new KeyValue();
				vx.setKey("W Angle");
				vx.setValue(String.valueOf(visionAngle));
				orient.add(vx);
				orient.add(vy);
				orient.add(vz);
				orient.add(vw);
				tagmsg.setMessage(visionName);
				tagmsg.setValues(orient);
				*/
				tagmsg.setX(visionX);
				tagmsg.setY(visionY);
				tagmsg.setZ(visionDistance);
				tagmsg.setW(visionAngle);
				
			}
			tagpub.publish(tagmsg);
			//statpub.publish(tagmsg);
			isVision = false;
		}
		
		Thread.sleep(1);

	}
	
}); // cancellable loop

System.out.println("Leaving cancellable publishing loop");
}


					//System.out.println("Pitch: " + pitch + " Roll: " + roll);
	/*
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
	    */
		


	// This CancellableLoop will be canceled automatically when the node shuts
	// down.
	/*
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
		}
	});  
*/

}


