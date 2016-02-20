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

import com.neocoretechs.robocore.MotionController;
import com.neocoretechs.robocore.MotorControl;
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

	IDrone drone;
	//double phi, theta, psi;
	geometry_msgs.Point32 rangeTop, rangeBottom; // Ultrasonic sensors, one is external to ARDrone and sits on the bus as robocore/range
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
	int visionDistance, visionAngle, visionX, visionY;
	Time tst;
	int imwidth = 672, imheight = 418;

	
	ArrayBlockingQueue<byte[]> vidbuf = new ArrayBlockingQueue<byte[]>(128);
	Object vidMutex = new Object();
	Object navMutex = new Object();
	Object rngMutex = new Object();
	Object visMutex = new Object();

	public static float[] SHOCK_BASELINE = { 971.0f, 136.0f, 36.0f};
	public static float[] SHOCK_THRESHOLD = {1000.0f,1000.0f,1000.0f}; // deltas. 971, 136, 36 relatively normal values. seismic: last value swings from rangeTop -40 to 140
	public static boolean isShock = false;
	public static short[] MAG_THRESHOLD = {-1,-1,-1};
	public static boolean isMag = false;
	public static int PRESSURE_THRESHOLD = 100000; // pressure_meas is millibars*100 30in is 1014 milli
	public static boolean isPressure = false;
	public static long lastPressureNotification = 0; // time so we dont just keep yapping about the weather
	public static int TEMPERATURE_THRESHOLD = 50000; // C*1000 122F
	public static boolean isTemperature = false;
	public static boolean isMoving = false;
	public static boolean isVision = false; // machine vision recognition event
	
@Override
public GraphName getDefaultNodeName() {
	return GraphName.of("ardrone");
}

/**
 * Start the main processing pipeline
 */
@Override
public void onStart(final ConnectedNode connectedNode) {
	rangeTop = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Point32._TYPE);
	rangeBottom = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Point32._TYPE);	
	final Log log = connectedNode.getLog();
	//Subscriber<geometry_msgs.Twist> subsmotion = connectedNode.newSubscriber("cmd_vel", geometry_msgs.Twist._TYPE);
	Subscriber<std_msgs.Empty> substol = connectedNode.newSubscriber("ardrone/activate", std_msgs.Empty._TYPE);
	Subscriber<std_msgs.Empty> subsreset = connectedNode.newSubscriber("ardrone/reset", std_msgs.Empty._TYPE);
	// Emergency stop message
	Subscriber<std_msgs.Empty> subschannel = connectedNode.newSubscriber("ardrone/zap", std_msgs.Empty._TYPE);
	// Ultrasonic sensor independant of ARDrone, rolled into point cloud range pub
	Subscriber<sensor_msgs.Range> subsrange = connectedNode.newSubscriber("robocore/range", sensor_msgs.Range._TYPE);
	
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
	final Publisher<sensor_msgs.PointCloud> rangepub = 
		connectedNode.newPublisher("ardrone/range", sensor_msgs.PointCloud._TYPE);
	// statpub has status alerts that may come from ARDrone extreme attitude, temp etc.
	final Publisher<diagnostic_msgs.DiagnosticStatus> statpub =
			connectedNode.newPublisher("robocore/status", diagnostic_msgs.DiagnosticStatus._TYPE);
	
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
						rangeTop.setX(ud.getRaw());
					}
				}
			}
			@Override
			public void receivedAltitude(int altitude) {
				synchronized(rngMutex) {
					//System.out.println("Altitude: "+altitude);
					if( altitude != 0 ) {
						rangeTop.setX(altitude);
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
					//System.out.println("Shock:"+accs[0]+" "+accs[1]+" "+accs[2]);
					if( SHOCK_THRESHOLD[0] != -1 ) {
						if( Math.abs(accs[0]-SHOCK_BASELINE[0]) > SHOCK_THRESHOLD[0] ) {
							isShock = true;
							return;
						}
						if( Math.abs(accs[1]-SHOCK_BASELINE[1]) > SHOCK_THRESHOLD[1] ) {
							isShock = true;
							return;
						}
						if( Math.abs(accs[2]-SHOCK_BASELINE[2]) > SHOCK_THRESHOLD[2] ) {
							isShock = true;
							return;
						}
					}
				}
				//System.out.println("Phys Accelero:"+d);
			}
	
		});
		
		drone.getNavDataManager().addMagnetoListener(new MagnetoListener() {
			@Override
			public void received(MagnetoData d) {
				//System.out.println("Mag:"+d);
				short mag[] = d.getM();
				if( MAG_THRESHOLD[0] != -1 ) {
					if( mag[0] > MAG_THRESHOLD[0] ) {
						isMag = true;
						return;
					}
					if( mag[1] > MAG_THRESHOLD[1] ) {
						isMag = true;
						return;
					}
					if( mag[2] > MAG_THRESHOLD[2] ) {
						isMag = true;
						return;
					}		
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
				//System.out.println("Pressure:"+d);
				if( PRESSURE_THRESHOLD != -1 && PRESSURE_THRESHOLD > d.getMeasurement() ) { // check for dropping
					long meas = System.currentTimeMillis()-lastPressureNotification;
					if( meas > 1000000) {
						isPressure = true;
					}
					return;
				}
				
			}
	
		});
		
		drone.getNavDataManager().addTemperatureListener(new TemperatureListener() {
			@Override
			public void receivedTemperature(Temperature d) {
				//System.out.println("Temp:"+d);
				if( TEMPERATURE_THRESHOLD != -1 && TEMPERATURE_THRESHOLD < d.getValue() ) {
					isTemperature = true;
					return;
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
	 * Receives the ultrasonic range from supplimental ranger(s)
	 */
	subsrange.addMessageListener(new MessageListener<sensor_msgs.Range>() {
		@Override
		public void onNewMessage(Range message) {
			synchronized(rngMutex) {
				rangeBottom.setX(message.getRange());
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
	
			//So far I've been unable to figure out how to fill the K and P matrices
			//using rosjava --J.Pablo
			//double[] K = {imwidth/2.0, 0, imwidth/2.0, 0, 160, 120, 0, 0, 1};
			//double[] P = {160, 0, 160, 0, 0, 160, 120, 0, 0, 0, 1, 0};

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
			
			navpub.publish(str);
			//System.out.println("Pub nav:"+str);
			Thread.sleep(1);
			
			// Generate point cloud data which will include
			// ultrasonic range info and any other ranging we field
			sensor_msgs.PointCloud rangemsg = rangepub.newMessage();
			//rangemsg.setFieldOfView(35);
			//rangemsg.setMaxRange(6000);
			//rangemsg.setMinRange(0);
			//rangemsg.setRadiationType(sensor_msgs.Range.ULTRASOUND);
			synchronized(rngMutex) {
					//rangemsg.setRange(rangeTop);
				List<geometry_msgs.Point32> value = new ArrayList<geometry_msgs.Point32>();
				value.add(rangeTop);
				value.add(rangeBottom);
				rangemsg.setPoints(value);
			}
			rangepub.publish(rangemsg);
			//System.out.println("Pub rangeTop:"+rangemsg);
			Thread.sleep(1);
			
			if( isShock || isMag || isPressure || isTemperature ) {
				diagnostic_msgs.DiagnosticStatus statmsg = statpub.newMessage();
				if(isShock && !isMoving) {
					isShock = false;
					statmsg.setName("shock");
					statmsg.setLevel(diagnostic_msgs.DiagnosticStatus.WARN);
					statmsg.setMessage("Accelerometer shock warning");
					diagnostic_msgs.KeyValue kv = connectedNode.getTopicMessageFactory().newFromType(diagnostic_msgs.KeyValue._TYPE);
					List<diagnostic_msgs.KeyValue> li = new ArrayList<diagnostic_msgs.KeyValue>();
					li.add(kv);
					statmsg.setValues(li);
					statpub.publish(statmsg);
					Thread.sleep(1);
				}
			
				if(isMag && !isMoving) {
					isMag = false;
					statmsg.setName("magnetic");
					statmsg.setLevel(diagnostic_msgs.DiagnosticStatus.WARN);
					statmsg.setMessage("Magnetic anomaly detected");
					diagnostic_msgs.KeyValue kv = connectedNode.getTopicMessageFactory().newFromType(diagnostic_msgs.KeyValue._TYPE);
					List<diagnostic_msgs.KeyValue> li = new ArrayList<diagnostic_msgs.KeyValue>();
					li.add(kv);
					statmsg.setValues(li);
					statpub.publish(statmsg);
					Thread.sleep(1);
				}
			
				if(isPressure) {
					isPressure = false;
					statmsg.setName("pressure");
					statmsg.setLevel(diagnostic_msgs.DiagnosticStatus.WARN);
					statmsg.setMessage("Atmospheric pressure warning");
					diagnostic_msgs.KeyValue kv = connectedNode.getTopicMessageFactory().newFromType(diagnostic_msgs.KeyValue._TYPE);
					List<diagnostic_msgs.KeyValue> li = new ArrayList<diagnostic_msgs.KeyValue>();
					li.add(kv);
					statmsg.setValues(li);
					statpub.publish(statmsg);
					Thread.sleep(1);
				}
			
				if(isTemperature) {
					isTemperature = false;
					statmsg.setName("temprature");
					statmsg.setLevel(diagnostic_msgs.DiagnosticStatus.WARN);
					statmsg.setMessage("Temperature warning");
					diagnostic_msgs.KeyValue kv = connectedNode.getTopicMessageFactory().newFromType(diagnostic_msgs.KeyValue._TYPE);
					List<diagnostic_msgs.KeyValue> li = new ArrayList<diagnostic_msgs.KeyValue>();
					li.add(kv);
					statmsg.setValues(li);
					statpub.publish(statmsg);
					Thread.sleep(1);
				}
			} // isShock, isMag...
	
		}
		
	}); // cancellable loop
}


}
