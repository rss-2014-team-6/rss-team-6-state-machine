package StateMachine;

import map.CSpace;
import map.PolygonMap;
import map.PolygonObstacle;

import org.ros.message.MessageListener;

import rss_msgs.PositionMsg;
import rss_msgs.VelocityMsg;
import rss_msgs.PositionTargetMsg;
import rss_msgs.WaypointMsg;
import rss_msgs.BumpMsg;
import rss_msgs.BreakBeamMsg;
import rss_msgs.SonarMsg;
import rss_msgs.InitializedMsg;
import rss_msgs.BallLocationMsg;
import rss_msgs.MapMsg;
import rss_msgs.OdometryMsg;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import java.util.Random;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.lang.InterruptedException;

/**
 * 
 * @author cwetters
 * 
 */
public class StateMachine extends AbstractNodeMain implements Runnable {

    private Subscriber<PositionMsg> posSub;
    private Subscriber<WaypointMsg> waypointSub;
    private Subscriber<BumpMsg> bumpSub;
    private Subscriber<BreakBeamMsg> breakbeamSub;
    private Subscriber<SonarMsg> sonarSub;
    private Subscriber<BallLocationMsg> ballLocationSub;
    private Subscriber<MapMsg> mapSub;
    
    private Publisher<PositionTargetMsg> posTargMsgPub;
    private Publisher<std_msgs.String> ctrlStatePub;
    private Publisher<VelocityMsg> velPub;
    private Publisher<InitializedMsg> initPub;
    private Publisher<PositionTargetMsg> motorsPub;
    private Publisher<BallLocationMsg> ballLocationPub;

    private Publisher<WaypointMsg> waypointPub;
    
    private Publisher<PositionMsg> posPub;
    private Subscriber<OdometryMsg> odoSub;
    
    private PositionTargetMsg currGoal;
    private WaypointMsg currWaypoint;
    
    private CSpace cSpace;
    private PolygonMap map;
    
    private final double ACCEPTABLE_ERROR = 0.05;
    private static final double ROBOT_RADIUS = 0.3;
    private final double HOME_X = 0.0;
    private final double HOME_Y = 0.0;
    private final double HOME_THETA = 0.0;
    
    private long startTime;
    private long lastBump;
    
    private double myX;
    private double myY;
    private double myTheta;

    private final long WANDER_TIME = 420000; //7 minutes 
    
    private State lastState;
    private State state;
    
    /*  The start state.  Should be pretty self explanatory.
     */
    private State startState = new State("start"){
	    @Override public void handle (BumpMsg msg){
		VelocityMsg vmsg = velPub.newMessage();
		vmsg.setTranslationVelocity(0);
		vmsg.setRotationVelocity(0);
		velPub.publish(vmsg);
		System.out.println("curr time: " + getTime());
		if(getTime() >= 10000) { // wait 10 seconds
                    InitializedMsg imsg = initPub.newMessage();
                    imsg.setInitialized(true);
                    initPub.publish(imsg);
                    state = spin;
                    lastState = this;
                }
	    }
	};
    
    
    /**
     * States for Gather phase:
     * lost - find yourself. Only goes to whatever you were doing last/spin
     * spin - look around for block. goes to visualServo on a BallLocationMsg; after 2 spins goes to wander
     * visualServo - visualServo to a block. Goes to spin or drive depending on where it came from
     * wander - wander to a new location. Goes to visualServo on a BallLocationMsg
     * 
     * exit states:
     * if enough time has passed, and if the robot is in spin or drive, go to driveEnter state 
     */
    private State lost = new State("lost"){
	    @Override
		public void handle(PositionMsg msg){
		if(localized(msg)){
		    state = lastState;
		    lastState = this;
		}
	    }
	};
    
    private State spin = new State("spin"){
	    private long spins_start = -1;
	    private final long SPIN_TIME = 20000;
      
	    @Override
		public void handle(PositionMsg msg){
		if (!localized(msg)){
		    state = lost;
		    lastState = this;
		    return;
		}
		if (timeToGoHome()){
		    state = driveBuildSite;
		    lastState = this;
		    PositionTargetMsg targmsg = posTargMsgPub.newMessage();
		    targmsg.setX(HOME_X);
		    targmsg.setY(HOME_Y);
		    targmsg.setTheta(HOME_THETA);
		    currGoal = targmsg;
		    posTargMsgPub.publish(targmsg);
		    return;
		}
		if (spins_start == -1){
		    spins_start = getTime();
		    VelocityMsg vmsg = velPub.newMessage();
		    vmsg.setTranslationVelocity(0);
		    vmsg.setRotationVelocity(2.0);
		    velPub.publish(vmsg);

		}
		if (getTime() - spins_start > SPIN_TIME){ 
		    VelocityMsg vmsg = velPub.newMessage();
		    vmsg.setTranslationVelocity(0);
		    vmsg.setRotationVelocity(0);
		    velPub.publish(vmsg);
		    spins_start = -1;
		    state = wander;
		    lastState = this;
		    publishWander();
		}
	    }
      
	    @Override
		public void handle(BallLocationMsg msg){
		state = visualServo;
		lastState = this;
		spins_start = -1;
		state.handle(msg);
                  
	    }
      
	    @Override
		public void handle(BumpMsg msg){
		if (msg.getLeft() || msg.getRight()) {
		    spins_start = -1;
		    lastBump = getTime();
		    state = bumped;
		    lastState = this;
		}
	    }
	};
    
    private State visualServo = new State("visualServo"){      
	    private final double PICKUP_THRESHOLD = .02;
	    @Override
		public void handle(BallLocationMsg msg){   
		WaypointMsg way = waypointPub.newMessage();
	   
		way.setX(myX + (msg.getRange()+ 0.3)*Math.cos(myTheta + msg.getBearing())); //aim a bit behind the block? 
		way.setY(myY + (msg.getRange()+ 0.3)*Math.sin(myTheta + msg.getBearing())); 
		way.setTheta(-1);
		waypointPub.publish(way);
		currWaypoint = way;

		//TODO
		//check if it's the same block
           
	    }

	    @Override
		public void handle(PositionMsg msg){
		if(Math.sqrt(Math.pow(currWaypoint.getX() - msg.getX(), 2) + Math.pow(currWaypoint.getY() - msg.getY(), 2)) < PICKUP_THRESHOLD){
		    state = lastState;
		    lastState = this;
		}
	    }
       
	    @Override
		public void handle(BumpMsg msg){
		if (msg.getLeft() || msg.getRight()) {
		    lastBump = getTime();
		    state = bumped;
		    lastState = this;
		}
	    }
	};
    
    private State wander = new State("wander"){
	    final double WANDER_THRESHOLD = 0.05;

	    @Override
		public void handle(PositionMsg msg){
		if (!localized(msg)){
		    state = lost;
		    lastState = this;
		    return;
		}
		if (timeToGoHome()){
		    state = driveBuildSite;
		    lastState = this;
		    PositionTargetMsg targmsg = posTargMsgPub.newMessage();
		    targmsg.setX(HOME_X);
		    targmsg.setY(HOME_Y);
		    targmsg.setTheta(HOME_THETA);
		    currGoal = targmsg;
		    posTargMsgPub.publish(targmsg);
		    return;
		}
		posTargMsgPub.publish(currGoal);
		if(dist(msg) < WANDER_THRESHOLD){
		    state = spin;
		    lastState = this;
		}
	    }
        
	    @Override 
		public void handle(BallLocationMsg msg){
		state = visualServo;
		lastState = this;
		publishWander();
		state.handle(msg);
	    }
        
	    @Override
		public void handle(WaypointMsg msg){
		waypointPub.publish(msg);
	    }
        
	    @Override
		public void handle(BumpMsg msg){
		if (msg.getLeft() || msg.getRight()) {
		    lastBump = getTime();
		    state = bumped;
		    lastState = this;
		    publishWander();
		}
	    }
	};
    
    private State bumped = new State("bumped"){
	    private final double BUMP_TIME = 2000;
	    @Override
		public void handle(PositionMsg msg){
		if(getTime() - lastBump > BUMP_TIME){
		    state = spin;
		    lastState = this;
		    VelocityMsg vmsg = velPub.newMessage();
		    vmsg.setTranslationVelocity(0);
		    vmsg.setRotationVelocity(0);
		    velPub.publish(vmsg);
		    state.handle(msg);
		}
		VelocityMsg vmsg = velPub.newMessage();
		vmsg.setTranslationVelocity(-2.0);
		vmsg.setRotationVelocity(0);
		velPub.publish(vmsg);
	    }
	};
    
   
    
    /**
     * States for Drive to build site
     * driveBuildSite - drives home. 
     * driveLost - find yourself. Goes back to driveBuildSite
     * 
     * 
     * exit states:
     * when you're at the build site go to buildXX
     */
    private State driveBumped = new State("driveBumped"){
	    private final double BUMP_TIME = 2000;
	    @Override
		public void handle(PositionMsg msg){
		if(getTime() - lastBump > BUMP_TIME){
		    state = driveBuildSite;
		    lastState = this;
		    VelocityMsg vmsg = velPub.newMessage();
		    vmsg.setTranslationVelocity(0);
		    vmsg.setRotationVelocity(0);
		    velPub.publish(vmsg);
		}
		VelocityMsg vmsg = velPub.newMessage();
		vmsg.setTranslationVelocity(-2.0);
		vmsg.setRotationVelocity(0);
		velPub.publish(vmsg);
	    }
	};
    
    private State driveLost = new State ("driveLost"){
	    @Override
		public void handle(PositionMsg msg){
		if(localized(msg)){
		    state = lastState;
		    lastState = this;
		}
	    }
	};
    
    private State driveBuildSite = new State("driveBuildSite"){
	    final double HOME_THRESHOLD = 0.5;

	    @Override
		public void handle(PositionMsg msg){
		if (!localized(msg)){
		    state = driveLost;
		    lastState = this;
		    return;
		}
		posTargMsgPub.publish(currGoal);
		if (dist(msg) < HOME_THRESHOLD){
		    state = buildEnter;
                
		    openFlap();
		}
	    }
        
	    @Override 
		public void handle(WaypointMsg msg){
		waypointPub.publish(msg);
	    }
        
	    @Override
		public void handle(BumpMsg msg){
		if (msg.getLeft() || msg.getRight()) {
		    lastBump = getTime();
		    state = driveBumped;
		    lastState = this;
		}
	    }
        
	};
    
    /**
     * States for building stuff: what needs to happen here?
     * drive forward til you bump the wall, then stop. 
     */
    
    private State buildEnter = new State("buildEnter"){
	    private boolean stop = false;
	    @Override
		public void handle(PositionMsg msg){
		if (!stop){
		    VelocityMsg vmsg = velPub.newMessage();
		    vmsg.setTranslationVelocity(2.0);
		    vmsg.setRotationVelocity(0);
		    velPub.publish(vmsg);
		} else {
		    VelocityMsg vmsg = velPub.newMessage();
		    vmsg.setTranslationVelocity(0);
		    vmsg.setRotationVelocity(0);
		    velPub.publish(vmsg);
		}
	    }
	    @Override
		public void handle(BumpMsg msg){
		if (msg.getLeft() || msg.getRight()) {
		    stop = true;
		    // we want to stop as soon as we hit the wall
		    VelocityMsg vmsg = velPub.newMessage();
		    vmsg.setTranslationVelocity(0);
		    vmsg.setRotationVelocity(0);
		    velPub.publish(vmsg);
		    lastState=this;
		    state = buildDriveBack;
		}
	    }
	}; 

    private State buildDriveBack = new State("buildDriveBack"){
	    private int doneTime = 5000;  // drive back for 5 seconds
	    private int startTime = -1;
	    @Override
		public void handle(PositionMsg msg){
		if (startTime == -1)
		    startTime = getTime();
		if (getTime() < doneTime){
		    //drive backwards
		    VelocityMsg vmsg = velPub.newMessage();
		    vmsg.setTranslationVelocity(-2.0);
		    vmsg.setRotationVelocity(0);
		    velPub.publish(vmsg);
		} else {
		    // we're done! stop.
		    VelocityMsg vmsg = velPub.newMessage();
		    vmsg.setTranslationVelocity(0);
		    vmsg.setRotationVelocity(0);
		    velPub.publish(vmsg);

		    lastState = this;
		    state = done;
		}
	    }
	    
	};
    
    private State done = new State("done"){
	    
	};

    
    public void openFlap(){
        //@TODO make flap open...
        
    }
    
    public boolean localized(PositionMsg msg){
        //@TODO:check if msg has enough confidence to mean we are sure we know where we are
        //@TODO: keep track of position updates, check to see if positon is changing to rapidly 
        return true;
    }
    
    public double dist(PositionMsg odo){
        // not sure how you want to deal with this more nicely, this is hackish --bhomberg
        if(currGoal == null)
            return Double.POSITIVE_INFINITY;
	return Math.sqrt(Math.pow(odo.getX() - currGoal.getX(), 2) + 
			 Math.pow(odo.getY() - currGoal.getY(), 2));
            
    }
    
    /**
     * Get time since start of execution in ms.
     */
    public long getTime(){
        return System.currentTimeMillis() - startTime;
    }
    
    public boolean timeToGoHome(){
        return getTime() > WANDER_TIME;
    }

    private void publishWander(){
        boolean validGoal = false;
        PositionTargetMsg msg = posTargMsgPub.newMessage();
        
    	if (map != null) {
    	    Rectangle2D worldRect = map.getWorldRect();
    	    while(!validGoal){
		double x = Math.random()*worldRect.getWidth() + worldRect.getX();
		double y = Math.random()*worldRect.getHeight() + worldRect.getY();
		validGoal = map.isValid(x, y);
		if (validGoal) {
		    msg.setX(x);
		    msg.setY(y);
		    break;
		}
    	    }
    		              
            msg.setTheta(-1);
    
            
    	    currGoal = msg;
    	    posTargMsgPub.publish(msg);
    	}
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////end States
    
    
    private Random rand; // for testing --bhomberg
    
    private int count;
   
    ////////////////////////////////////////////////////////////////////////////////handlers

    private void handle(MapMsg msg) {
        try {
            byte[] ba = msg.getSerializedMap().array();
            ByteArrayInputStream byteStream = new ByteArrayInputStream(ba);
            // Skip the 4-byte length header
            byteStream.skip(4);
            
            ObjectInputStream stream = new ObjectInputStream(byteStream);

            map = (PolygonMap) stream.readObject();
            map.recalculateCSpace();
            stream.close();
        }
        catch (IOException e) {
	    throw new RuntimeException ("IOException in handleMapMsg");
            //e.printStackTrace();
            //return;
        }
        catch (ClassNotFoundException e) {
	    throw new RuntimeException ("ClassNotFoundException in handleMapMsg");
            //e.printStackTrace();
            //return;
        }
    }
    
    public void handle(PositionMsg odo){
        //IMPLEMENT_STATES
        myX = odo.getX();
        myY = odo.getY();
        myTheta = odo.getTheta();
        state.handle(odo);   

	//publish what state we're in for debugging purposes
	std_msgs.String msg = ctrlStatePub.newMessage();
	msg.setData(state.getName());
	ctrlStatePub.publish(msg);
    }
    
    public void handle(WaypointMsg way){
        //IMPLEMENT_STATES
        state.handle(way);
        
    }
    
    public void handle(BumpMsg bump){
        //IMPLEMENT_STATES
        state.handle(bump);
       
    }
    public void handle(BreakBeamMsg bbeam){
	//IMPLEMENT_STATES
        state.handle(bbeam);
       
    }
    public void handle(SonarMsg sonar){
        //IMPLEMENT_STATES
        state.handle(sonar);
    }
    public void handle(BallLocationMsg ball){
        state.handle(ball);
    }
   

    @Override
	public void run() {
	//this used to do visiony things, idk what it's supposed to do now. 
	   
    }
    
    
    /**
     * <p>
     * Run the VisualServo process
     * </p>
     * 
     * @param node
     *            optional command-line argument containing hostname
     */
    @Override
	public void onStart(final ConnectedNode node) {
    	System.out.println("Hi, I'm a state machine!");
    	
	startTime = System.currentTimeMillis();
    
    	//posPub = node.newPublisher("im/alsofake","rss_msgs/PositionMsg");
    	
    	posTargMsgPub = node.newPublisher("/state/PositionTarget", "rss_msgs/PositionTargetMsg");
    	ctrlStatePub = node.newPublisher("/state/State", std_msgs.String._TYPE);
    	waypointPub = node.newPublisher("/state/Waypoint", "rss_msgs/WaypointMsg");
    
    	velPub = node.newPublisher("/state/Velocity", "rss_msgs/VelocityMsg");
	initPub = node.newPublisher("/state/Initialized", "rss_msgs/InitializedMsg");
    	
        ballLocationPub = node.newPublisher("/state/BallLocation", "rss_msgs/BallLocationMsg");

    	/*odoSub = node.newSubscriber("/odo/Odometry", "rss_msgs/OdometryMsg");
    	odoSub.addMessageListener(new MessageListener<rss_msgs.OdometryMsg>(){
        	@Override
		    public void onNewMessage(OdometryMsg msg){
		    //running off of localization position rather than odometry position
		    PositionMsg pos = posPub.newMessage();
		      pos.setX(msg.getX());
		      pos.setY(msg.getY());
		      pos.setTheta(msg.getTheta());
		      handle(pos);	
        	}	
        	
	    });*/

        posSub = node.newSubscriber("/loc/Position", "rss_msgs/PositionMsg");
        posSub.addMessageListener(new MessageListener<rss_msgs.PositionMsg>() {
		@Override
		    public void onNewMessage(rss_msgs.PositionMsg message) {
		    handle(message);
		}
	    });
    
    
    
        waypointSub = node.newSubscriber("/path/Waypoint", "rss_msgs/WaypointMsg");
        waypointSub.addMessageListener(new MessageListener<rss_msgs.WaypointMsg>(){
		@Override
		    public void onNewMessage(rss_msgs.WaypointMsg message){
		    handle(message);
		}
	    });
        
        
        bumpSub = node.newSubscriber("/sense/Bump", "rss_msgs/BumpMsg");
        bumpSub.addMessageListener(new MessageListener<rss_msgs.BumpMsg>(){
		@Override
		    public void onNewMessage(rss_msgs.BumpMsg message){
		    handle(message);
		}
	    });
        
        breakbeamSub = node.newSubscriber("/sense/BreakBeam", "rss_msgs/BreakBeamMsg");
        breakbeamSub.addMessageListener(new MessageListener<rss_msgs.BreakBeamMsg>() {
		@Override
		    public void onNewMessage(rss_msgs.BreakBeamMsg message){
		    handle(message);
		}
	    });
        
        sonarSub = node.newSubscriber("/sense/Sonar", "rss_msgs/SonarMsg");
        sonarSub.addMessageListener(new MessageListener<rss_msgs.SonarMsg>(){
		@Override
		    public void onNewMessage(rss_msgs.SonarMsg message){
		    handle(message);
		}
	    });



        state = startState;
        
        ballLocationSub = node.newSubscriber("/vision/BallLocation", "rss_msgs/BallLocationMsg");
        ballLocationSub.addMessageListener(new MessageListener<BallLocationMsg>() {
		@Override
		    public void onNewMessage(BallLocationMsg message){
		    handle(message);
		}
	    });
        
        mapSub = node.newSubscriber("/loc/Map", "rss_msgs/MapMsg");
        mapSub.addMessageListener(new MessageListener<MapMsg>() {
		@Override
		    public void onNewMessage(MapMsg msg) {
		    handle(msg);
		}
	    });

    }
    
        

    @Override
	public GraphName getDefaultNodeName() {
        return GraphName.of("rss/statemachine");
    }
}
