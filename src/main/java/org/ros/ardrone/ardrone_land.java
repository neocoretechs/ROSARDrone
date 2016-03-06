/**
 * AR Drone driver for ROS
 * Based on the JavaDrone project and rosjava and ardrone_utd and the original Japanese version and some other
 * assorted code.
 * @author jg
 */
package org.ros.ardrone;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ArrayBlockingQueue;

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

import sensor_msgs.Range;
import de.yadrone.base.AbstractConfigFactory;
import de.yadrone.base.IDrone;
import de.yadrone.base.command.misc.DetectionType;
import de.yadrone.base.command.vision.VisionTagType;
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

import com.twilight.h264.decoder.AVFrame;
import com.twilight.h264.player.FrameUtils;
import com.twilight.h264.player.RGBListener;
/**
 * This class takes a series of messages demuxxed from the ARDrone and remuxxed onto the ROS bus.
 * In the case of IMU data, NavPacket(s) are constructed in response to cmd_vel topic Twist messages
 * and ultrasonic and other sensors thusly fused.
 * @author jg
 *
 */
public class ardrone_land extends AbstractNodeMain  {
	private boolean DEBUG = true;
	IDrone drone;
	//double phi, theta, psi;
	float rangeTop; // Ultrasonic sensor
	byte[] bbuf = null;// = new byte[320*240*3];
	boolean started = true;
	boolean videohoriz = true;
	boolean emergency = false;

	double pitch = 0.0d;
	double roll = 0.0d;
	double yaw = 0.0d;
	double vertvel = 0.0d;
	double phi = 0.0d;
	double theta = 0.0d;
	double psi = 0.0d;
	
	float[] accs = new float[3]; // accelerometer values
	float gyros[] = new float[3]; // gyro
	short mag[] = new short[3]; // magnetometer
	short temperature = 0;
	int pressure = 0;
	int visionDistance, visionAngle, visionX, visionY;
	
	boolean isTemp = true;
	boolean isPress = true;
	boolean isTag = false;
	boolean isVision = false;
	
	Time tst;
	int imwidth = 672, imheight = 418;

	ArrayBlockingQueue<byte[]> vidbuf = new ArrayBlockingQueue<byte[]>(128);
	
	Object vidMutex = new Object();
	Object navMutex = new Object();
	Object rngMutex = new Object();
	Object visMutex = new Object();
	Object magMutex = new Object();
	Object tmpMutex = new Object();
	Object prsMutex = new Object();

	
@Override
public GraphName getDefaultNodeName() {
	return GraphName.of("ardrone");
}

/**
 * Start the main processing pipeline. We subscribe to an external ranging message robocore/range to augment the
 * ultrasonic range and supply a complete point cloud to the ardrone/range channel. If we have ultrasonics
 * they will be in the first elements of the point cloud array in X in addition to element 0 X being the ARDrone ranger
 */
@Override
public void onStart(final ConnectedNode connectedNode) {

	final Log log = connectedNode.getLog();
	//Subscriber<geometry_msgs.Twist> subsmotion = connectedNode.newSubscriber("cmd_vel", geometry_msgs.Twist._TYPE);
	Subscriber<std_msgs.Empty> substol = connectedNode.newSubscriber("ardrone/activate", std_msgs.Empty._TYPE);
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
	final Publisher<sensor_msgs.CameraInfo> caminfopub =
		connectedNode.newPublisher("ardrone/camera_info", sensor_msgs.CameraInfo._TYPE);
	// rangepub has point cloud data of ultrasonic and other range finders all rolled into one, 
	// assume first 2 points upper and lower ultrasonics
	final Publisher<sensor_msgs.Range> rangepub = 
		connectedNode.newPublisher("ardrone/range", sensor_msgs.Range._TYPE);
	// statpub has status alerts that may come from ARDrone extreme attitude, temp etc.
	//final Publisher<diagnostic_msgs.DiagnosticStatus> statpub =
	//		connectedNode.newPublisher("robocore/status", diagnostic_msgs.DiagnosticStatus._TYPE);
	final Publisher<sensor_msgs.Temperature> temppub = 
			connectedNode.newPublisher("ardrone/temperature", sensor_msgs.Temperature._TYPE);
	final Publisher<sensor_msgs.FluidPressure> presspub = 
			connectedNode.newPublisher("ardrone/pressure", sensor_msgs.FluidPressure._TYPE);
	final Publisher<sensor_msgs.MagneticField> magpub = 
			connectedNode.newPublisher("ardrone/magnetic_field", sensor_msgs.MagneticField._TYPE);
	final Publisher<geometry_msgs.Quaternion> tagpub = 
			connectedNode.newPublisher("ardrone/image_tag", geometry_msgs.Quaternion._TYPE);
	
	final Map<String, String> environment;


	try{
	
		drone = (IDrone)AbstractConfigFactory.createFactory("Land").createDrone();
		
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
		
	    drone.getVideoManager().addImageListener(new RGBListener() {
            public void imageUpdated(AVFrame newImage)
            {
            	int bufferSize;
            	synchronized(vidMutex) {
            		imwidth = newImage.imageWidth;
            		imheight = newImage.imageHeight;
            		bufferSize = imwidth * imheight * 3;
            	}	
				//if (bbuf == null || bufferSize != bbuf.capacity()) {
				//		bbuf = ByteBuffer.allocate(bufferSize);
				//}
				if( bbuf == null )
					bbuf = new byte[bufferSize];
				FrameUtils.YUV2RGB(newImage, bbuf); // RGBA32 to BGR8
				try {
					vidbuf.add(bbuf);
				} catch(IllegalStateException ise) {
					// buffer full;
					System.out.println("Video buffer full!");
					//vidbuf.clear();
				}
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

	substol.addMessageListener(new MessageListener<std_msgs.Empty>() {
	@Override
	public void onNewMessage(std_msgs.Empty message) {
		try
		{
			if(started)
			{
				drone.start();
				log.info("The robot is starting.");
			}
			else
			{
				drone.stop();
				log.info("The robot is stopping.");
			}
			started = !started;
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
					imwidth = 672;
					imheight = 418;
					log.info("Attempting to use the vertical camera.");
				} else {
					drone.setHorizontalCameraWithVertical();
					imwidth = 672;
					imheight = 418;
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
			imghead.setFrameId("0");

			//if( bbuf != null ) {
			byte[] bbuf = vidbuf.poll();
			if( bbuf != null ) {
				sensor_msgs.Image imagemess = imgpub.newMessage();
				sensor_msgs.CameraInfo caminfomsg = caminfopub.newMessage();
            	//System.out.println("Image:"+newImage.imageWidth+","+newImage.imageHeight+" queue:"+list.size());
				imagemess.setData(ChannelBuffers.wrappedBuffer(bbuf));
				imagemess.setEncoding("8UC3");
				synchronized(vidMutex) {
						imagemess.setWidth(imwidth);
						imagemess.setHeight(imheight);
						imagemess.setStep(imwidth*3);
						imagemess.setIsBigendian((byte)0);
						imagemess.setHeader(imghead);
						//
						caminfomsg.setHeader(imghead);
						caminfomsg.setWidth(imwidth);
						caminfomsg.setHeight(imheight);
						caminfomsg.setDistortionModel("plumb_bob");
				}
				//caminfomsg.setK(K);
				//caminfomsg.setP(P);
				imgpub.publish(imagemess);
				caminfopub.publish(caminfomsg);
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
			// Publish point cloud data to ardrone/range
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
			
			if( isVision) {
				geometry_msgs.Quaternion tagmsg = tagpub.newMessage();
				synchronized(visMutex) {
					tagmsg.setX(visionX);
					tagmsg.setY(visionY);
					tagmsg.setZ(visionDistance);
					tagmsg.setW(visionAngle);
				}
				tagpub.publish(tagmsg);
				isVision = false;
			}
			
			Thread.sleep(1);
	
		}
		
	}); // cancellable loop
}


}
