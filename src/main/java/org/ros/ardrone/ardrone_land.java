/**
 * AR Drone driver for ROS
 * Based on the JavaDrone project and rosjava and ardrone_utd and the original Japanese version and some other
 * assorted code.
 * @author jg
 */
package org.ros.ardrone;

import java.util.ArrayList;
import java.util.List;

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
import de.yadrone.base.IARDroneLand;
import de.yadrone.base.IDrone;
import de.yadrone.base.navdata.accel.AcceleroPhysData;
import de.yadrone.base.navdata.accel.AcceleroRawData;
import de.yadrone.base.navdata.data.Altitude;
import de.yadrone.base.navdata.data.KalmanPressureData;
import de.yadrone.base.navdata.data.MagnetoData;
import de.yadrone.base.navdata.data.Pressure;
import de.yadrone.base.navdata.data.Temperature;
import de.yadrone.base.navdata.listener.AcceleroListener;
import de.yadrone.base.navdata.listener.AltitudeListener;
import de.yadrone.base.navdata.listener.AttitudeListener;
import de.yadrone.base.navdata.listener.MagnetoListener;
import de.yadrone.base.navdata.listener.PressureListener;
import de.yadrone.base.navdata.listener.TemperatureListener;

import com.twilight.h264.decoder.AVFrame;
import com.twilight.h264.player.FrameUtils;
import com.twilight.h264.player.RGBListener;

public class ardrone_land extends AbstractNodeMain  {

	IDrone drone;
	double phi, theta, psi;
	int rangeTop, rangeBottom; // Ultrasonic sensors, one is external to ARDrone and sits on the bus as robocore/range
	byte[] bbuf;// = new byte[320*240*3];
	boolean started = true;
	boolean videohoriz = true;
	boolean emergency = false;
	double pitch, roll, yaw, vertvel;
	float[] accs; // accelerometer values
	Time tst;
	int imwidth = 672, imheight = 418;
	Object vidMutex = new Object(); 
	Object navMutex = new Object();
	Object rngMutex = new Object();
	public static float[] SHOCK_BASELINE = { 971.0f, 136.0f, 36.0f};
	public static float[] SHOCK_THRESHOLD = {100.0f,100.0f,100.0f}; // deltas. 971, 136, 36 relatively normal values. seismic: last value swings from rangeTop -40 to 140
	public static boolean isShock = false;
	public static short[] MAG_THRESHOLD = {-1,-1,-1};
	public static boolean isMag = false;
	public static int PRESSURE_THRESHOLD = 100000; // pressure_meas is millibars*100 30in is 1014 milli
	public static boolean isPressure = false;
	public static long lastPressureNotification = 0; // time so we dont just keep yapping about the weather
	public static int TEMPERATURE_THRESHOLD = 50000; // C*1000 122F
	public static boolean isTemperature = false;
	public static boolean isMoving = false;
	
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
	Subscriber<sensor_msgs.Range> subsrange = connectedNode.newSubscriber("robocore/rangeTop", sensor_msgs.Range._TYPE);
	
	final Publisher<geometry_msgs.Quaternion> navpub =
		connectedNode.newPublisher("ardrone/navdata", geometry_msgs.Quaternion._TYPE);
	final Publisher<sensor_msgs.Image> imgpub =
		connectedNode.newPublisher("ardrone/image_raw", sensor_msgs.Image._TYPE);
	final Publisher<sensor_msgs.CameraInfo> caminfopub =
		connectedNode.newPublisher("ardrone/camera_info", sensor_msgs.CameraInfo._TYPE);
	final Publisher<sensor_msgs.Range> rangepub = 
		connectedNode.newPublisher("ardrone/rangeTop", sensor_msgs.Range._TYPE);
	final Publisher<diagnostic_msgs.DiagnosticStatus> statpub =
			connectedNode.newPublisher("robocore/status", diagnostic_msgs.DiagnosticStatus._TYPE);

	
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
					phi = roll;
					theta = pitch;
					//System.out.println("Pitch: " + pitch + " Roll: " + roll);
					//((IARDroneLand)drone).move2D((int)phi,(float) theta);
				}
			}
			
			public void windCompensation(float pitch, float roll) { 
				synchronized(navMutex) {
					
				}
			}
		});
		
		drone.getNavDataManager().addAltitudeListener(new AltitudeListener() {
			public void receivedExtendedAltitude(Altitude ud) {
				synchronized(rngMutex) {
					//System.out.println("Ext. Alt.:"+ud);
					if( ud.getRaw() != 0 )
						rangeTop = ud.getRaw();
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
	
		/*
		drone.getNavDataManager().addBatteryListener(new BatteryListener() {	
			public void batteryLevelChanged(int percentage)
			{
				System.out.println("Battery: " + percentage + " %");
			}
			public void voltageChanged(int vbat_raw) { }
		});
		*/
		/*
		drone.getNavDataManager().addGyroListener(new GyroListener() {
			@Override
			public void receivedRawData(GyroRawData d) {
				//System.out.println("GyroRaw:"+d);	
			}
			@Override
			public void receivedPhysData(GyroPhysData d) {
				//System.out.println("GyroPhys:"+d);
				
			}
			@Override
			public void receivedOffsets(float[] offset_g) {
				//System.out.print("GyroOffs:");
				//for(float f: offset_g) System.out.print("offs:"+f+" ");
				//System.out.println();
			}
	
		});
		*/
	    
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
			// image width and height synamimcally set later, these are just setups
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
	 * Extract the linear and angular components from cmd_vel topic Twist quaternions, take the linear X (pitch) and
	 * angular Z (yaw) and send them to motor control. This results in motion planning computing a turn which
	 * involves rotation about a point in space located at a distance related to speed and angular velocity. The distance
	 * is used to compute the radius of the arc segment traversed by the wheel track to make the turn. If not moving we can
	 * make the distance 0 and rotate about a point in space, otherwise we must inscribe an appropriate arc. The distance
	 * in that case is the linear travel, otherwise the distance is the diameter of the arc segment.
	 * If we get commands on the cmd_vel topic we assume we are moving, if we do not get the corresponding IMU readings, we have a problem
	 * If we get a 0,0 on the X,yaw move we stop. If we dont see stable IMU again we have a problem, Houston.
	 */
	subscriber.addMessageListener(new MessageListener<geometry_msgs.Twist>() {
	@Override
	public void onNewMessage(geometry_msgs.Twist message) {
		geometry_msgs.Vector3 val = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
		val = message.getLinear();
		float targetRoll = (float) val.getY();
		float targetPitch = (float) val.getX();
		float targetVertvel = (float) val.getZ();
		val = message.getAngular();
		float targetYaw = (float) val.getZ();
		if( pitch == 0.0 && yaw == 0.0 )
				isMoving = false;
		else
				isMoving = true;
		try
		{
			//move2D(float yawIMURads, int yawTargetDegrees, int targetDistance, int targetTime)
			float taccs[];
			synchronized(accs) {
				taccs = accs.clone();
			}
			for(int i = 0; i < 3; i++)taccs[i] = Math.abs(taccs[i]-SHOCK_BASELINE[i]);
			int ranges[] = new int[2];
			ranges[0] = rangeTop;
			ranges[1] = rangeBottom;
			((IARDroneLand)drone).move2DRelative((float)yaw , (int)targetYaw, (int)targetPitch, 1, taccs, ranges);
		} catch (Throwable e) {
				e.printStackTrace();
		}  
		log.debug("Robot commanded to move:" + pitch + "mm linear in orientation " + yaw);
	}
	});

	subsrange.addMessageListener(new MessageListener<sensor_msgs.Range>() {
		@Override
		public void onNewMessage(Range message) {
			rangeBottom = (int) message.getRange();
		}
	});
	
	/**
	 * Main publishing loop. Essentially we are publishing the data in whatever state its in, using the
	 * mutex appropriate to establish critical sections. A 10ms sleep follows each publication to keep the bus arbitrated
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
			
			synchronized(vidMutex) {
			  	if( bbuf != null ) {
					sensor_msgs.Image imagemess = imgpub.newMessage();
            		//System.out.println("Image:"+newImage.imageWidth+","+newImage.imageHeight+" queue:"+list.size());
					imagemess.setData(ChannelBuffers.wrappedBuffer(bbuf));
					imagemess.setEncoding("8UC3");
					imagemess.setWidth(imwidth);
					imagemess.setHeight(imheight);
					imagemess.setStep(imwidth*3);
					imagemess.setIsBigendian((byte)0);
					imagemess.setHeader(imghead);
					sensor_msgs.CameraInfo caminfomsg = caminfopub.newMessage();
					caminfomsg.setHeader(imghead);
					caminfomsg.setWidth(imwidth);
					caminfomsg.setHeight(imheight);
					caminfomsg.setDistortionModel("plumb_bob");
					//caminfomsg.setK(K);
					//caminfomsg.setP(P);
					imgpub.publish(imagemess);
					caminfopub.publish(caminfomsg);
					//System.out.println("Pub cam:"+imagemess);
					sequenceNumber++;
			  	}
			}
			Thread.sleep(10);
			
			synchronized(navMutex) {
				geometry_msgs.Quaternion str = navpub.newMessage();
				str.setX(phi);
				str.setY(theta);
				str.setZ(rangeTop); // gaz
				str.setW(psi);
				navpub.publish(str);
				//System.out.println("Pub nav:"+str);
			}
			Thread.sleep(10);
			
			synchronized(rngMutex) {
				sensor_msgs.Range rangemsg = rangepub.newMessage();
				rangemsg.setFieldOfView(35);
				rangemsg.setMaxRange(6000);
				rangemsg.setMinRange(0);
				rangemsg.setRadiationType(sensor_msgs.Range.ULTRASOUND);
				rangemsg.setRange(rangeTop);
				rangepub.publish(rangemsg);
				//System.out.println("Pub rangeTop:"+rangemsg);
			}
			Thread.sleep(10);
			
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
					Thread.sleep(10);
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
					Thread.sleep(10);
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
					Thread.sleep(10);
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
					Thread.sleep(10);
				}
			}
		}
	}); // cancellable loop
}


}
