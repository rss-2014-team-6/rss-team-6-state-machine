package StateMachine;

//import java.util.Arrays;
//import java.util.concurrent.ArrayBlockingQueue;

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

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import java.util.Random;
import java.awt.geom.Point2D;
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
    //public Subscriber<WaypointMsg> waypointSub;
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
    

    //temporarily output waypoints so we can see if the motors move --bhomberg
    private Publisher<WaypointMsg> waypointPub;
    //end hacky
    
    
    private PositionTargetMsg currGoal;
    private WaypointMsg currWaypoint;
    
    private CSpace cSpace;
    private PolygonMap map;
    
    private final double ACCEPTABLE_ERROR = 0.05;
    private static final double ROBOT_RADIUS = 0.3;
    private final double HOME_X = 0.0;
    private final double HOME_Y = 0.0;
    
    private long startTime;
    
    private double myX;
    private double myY;
    private double myTheta;
    
    private int age;
    //private int state;
    
    /* private final int ERROR = -1;
    //overall time sections
    private final int GATHER = 1;
    private final int HOMEWARD = 2;
    private final int CONSTRUCT = 3;
    //gather states
    private final int LOST = 10;
    private final int VISUAL_SERVO = 11;
    private final int FIND_BLOCK = 12; //spin on state diagram pic
    private final int DRIVE_NEW_LOC = 13;
    //drive to build site states
    private final int DLOST = 20;
    private final int DRIVE_BUILD_SITE = 21;
    //build states
    private final int BLOST = 30;
    private final int BUILD1 = 31;
    private final int BUILD2 = 32;
    private final int BUILD3 = 33;
    private final int DRIVE_BUILD = 34; //drive to next place to build a tower*/

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
                    state = spinState;
                    lastState = this;
                }
	    }
	};
    
    private State spinState = new State ("spin"){
	    @Override public void handle(BumpMsg msg){
		if(getTime() <= 40000){ // spin for 30 s
		    VelocityMsg vmsg = velPub.newMessage();
		    vmsg.setTranslationVelocity(0);
		    vmsg.setRotationVelocity(2.0);
		    velPub.publish(vmsg);
		}
		else{
		    VelocityMsg vmsg = velPub.newMessage();
		    vmsg.setTranslationVelocity(0);
		    vmsg.setRotationVelocity(0);
		    velPub.publish(vmsg);
                    System.out.println("Transition to goal state");
		    state = goalState;
                    lastState = this;
		}

	    }
	};

    private State goalState = new State("goal"){
            @Override public void handle(BumpMsg msg){
                System.out.println("Goal state");
                PositionTargetMsg posTargMsg = posTargMsgPub.newMessage();
                posTargMsg.setX(-0.1);
                posTargMsg.setY(3.6);
                posTargMsg.setTheta(-1);
                posTargMsgPub.publish(posTargMsg);
                currGoal = posTargMsg;
            }
            @Override public void handle(PositionMsg msg){
                final double GOAL_THRESH = 0.03;
                if (dist(msg) < GOAL_THRESH) {
                    System.out.println("Reached goal!");
                    state = stopState;
                    lastState = this;
                }
            }
        };

    private State stopState = new State("stop"){
	};
	
//////////////////////////////////////////////////////////////////////////////////States below, not integrated stuff above
    private final long WANDER_TIME = 240000; //4 minutes
    
    private State lastState;
    private State state;
    
    
    private State example = new State ("example"){
      @Override
      public void handle (PositionMsg msg){
          System.out.println("I'm an overwritten state handler for pose");
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
      private final long SPIN_TIME = 5000;
      @Override
      public void handle(PositionMsg msg){
          if (!localized(msg)){
              state = lost;
              lastState = this;
              return;
          }
          if (timeToGoHome()){
              state = driveBuildSite;
              lastState = driveLost;
              PositionTargetMsg targmsg = posTargMsgPub.newMessage();
              targmsg.setX(HOME_X);
              targmsg.setY(HOME_Y);
              targmsg.setTheta(-1);
              currGoal = targmsg;
              posTargMsgPub.publish(targmsg);
              return;
          }
          if (spins_start == -1){
              spins_start = getTime();
          }
          if (getTime() - spins_start > SPIN_TIME){ 
                state = wander;
                lastState = this;
                publishWander();
          }
      }
      
      @Override
      public void handle(BallLocationMsg msg){
          state = visualServo;
          lastState = this;
          state.handle(msg);
                  
      }
    };
    
    private State visualServo = new State("visualServo"){
       @Override
       private final double PICKUP_THRESHOLD = .02;
       public void handle(BallLocationMsg msg){
           std_msgs.String ctrlState = ctrlStatePub.newMessage();
           ctrlState.setData("visualServo");
           ctrlStatePub.publish(ctrlState);
           
           WaypointMsg way = waypointPub.newMessage();
           way.setX(myX + msg.getBearing()*Math.cos(msg.getTheta())); //aim a bit behind the block? 
           way.setY(myY + msg.getBearing()*Math.sin(msg.getTheta())); 
           way.setTheta(-1);
           waypointPub.publish(way);
           currGoal = way;
           //check if it's the same block
           //check if ball is no longer in frame, drive forward extra x feet, return to previous state
           
       }
       
       public void handle(PositionMsg msg){
           if(distance(msg) < PICKUP_THRESHOLD){
               state = lastState;
               lastState = lost;
           }
       }
       
       
    };
    
    private State wander = new State("wander"){
        final double WANDER_THRESHOLD = 0.5;

        @Override
        public void handle(PositionMsg msg){
            if (!localized(msg)){
                state = lost;
                lastState = this;
                return;
            }
            if (timeToGoHome()){
                state = driveBuildSite;
                lastState = driveLost;
                PositionTargetMsg targmsg = posTargMsgPub.newMessage();
                targmsg.setX(HOME_X);
                targmsg.setY(HOME_Y);
                targmsg.setTheta(-1);
                currGoal = targmsg;
                posTargMsgPub.publish(targmsg);
                return;
            }
            
            if(Math.sqrt(Math.pow(msg.getX() - currGoal.getX(), 2) + Math.pow(msg.getY() - currGoal.getY(), 2))< WANDER_THRESHOLD){
                state = spin;
                lastState = lost;
            }
        }
        
        public void handle(BallLocationMsg msg){
            state = visualServo;
            lastState = this;
            state.handle(msg);
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
            if (Math.sqrt(Math.pow(HOME_X - msg.getX(), 2) + Math.pow(HOME_Y - msg.getY(),2)) < HOME_THRESHOLD){
                state = buildEnter;
                lastState = buildLost;
            }
        }
    };
    
    /**
     * States for building stuff: what needs to happen here?
     */
    
    private State buildEnter = new State("buildEnter"); //what are we gonna build?
    
    private State buildLost = new State("buldLost"); //does this need to exist?
    
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
        
        while(!validGoal){
            validGoal = true;
            msg.setX(Math.random()*3 - .5);
            msg.setY(Math.random()*4.5 - .5);
            Point2D.Double goal = new Point2D.Double(msg.getX(), msg.getY());
            for (PolygonObstacle obs : cSpace.getObstacles()){
                if(obs.contains(goal)){
                    validGoal = false;
                }
            }    
        }
        
        currGoal = msg;
        posTargMsgPub.publish(msg);    
    }
///////////////////////////////////////////////////////////////////////////////////////////////end States
    
    
    private Random rand; // for testing --bhomberg
    
    private int count;
   
////////////////////////////////////////////////////////////////////////////////handlers

    private void handleMapMsg(MapMsg msg) {
        try {
            byte[] ba = msg.getSerializedMap().array();
            ByteArrayInputStream byteStream = new ByteArrayInputStream(ba);
            // Skip the 4-byte length header
            byteStream.skip(4);
            
            ObjectInputStream stream = new ObjectInputStream(byteStream);

            map = (PolygonMap) stream.readObject();
            stream.close();
        System.out.println("Initialized");
            initialized = true;
            cSpace = new CSpace(map.getObstacles(), ROBOT_RADIUS);
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
        /**
        myX = odo.getX();
        myY = odo.getY();
        myTheta = odo.getTheta();
        state.handle(odo);
       **/
        if(dist(odo) < ACCEPTABLE_ERROR){
            PositionTargetMsg msg = posTargMsgPub.newMessage();
            msg.setX(Math.random()*5.0);
            msg.setY(Math.random()*5.0);
            posTargMsgPub.publish(msg);
            currGoal = msg;
        
        }
        System.out.println("odo message handled x: " + odo.getX() + 
                                                "  y: " + odo.getY() + 
                                                "  theta: " + odo.getTheta());
    }
    
    public void handle(WaypointMsg way){
        //IMPLEMENT_STATES
        //state.handle(way);
        
        System.out.println("waypoint message handled x: " + way.getX() +
                                                        "  y: " + way.getY() +
                                                        "  theta: " + way.getTheta());
        //send waypoint since motion isn't updated yet and I don't know what it's going to be
        /**
        PositionTargetMsg msg = motorsPub.newMessage();
        msg.setX(way.getX());
        msg.setY(way.getY());
        msg.setTheta(way.getTheta());
        motorsPub.publish(msg);
	*/
        //end hacks
    }
    
    public void handle(BumpMsg bump){
        //IMPLEMENT_STATES
        //state.handle(bump);
        
        System.out.println("bump message handled");
	count++;

	if(count >= 20){
	    PositionTargetMsg msg = posTargMsgPub.newMessage();
	    msg.setX(-0.1524);
	    msg.setY(3.6576);
	    //msg.setX(rand.nextDouble()*3 - 0.5);
	    //msg.setY(rand.nextDouble()*4.5 - 0.5);
	    msg.setTheta(-1);
	    posTargMsgPub.publish(msg);
	    count = 0;
	}
    }
    public void handle(BreakBeamMsg bbeam){
         //IMPLEMENT_STATES
        //state.handle(bbeam);
        System.out.println("bbeam handled");
    }
    public void handle(SonarMsg sonar){
        //IMPLEMENT_STATES
        //state.handle(sonar);
        System.out.println("sonar handled");
    }
////////////////////////////////////////////////////////////////////////////////end handlers
   

    @Override
    public void run() {
        /*while (true) {
	    System.out.println("In SM run loop");
	    //this used to do visiony things, idk what it's supposed to do now. 
	    
	    // testing -- output new random waypoints every 2 seconds --bhomberg
	    try{
		Thread.sleep(2000);
	    } catch (InterruptedException e) {
		System.out.println("Thread sleeping is sadface. :( ");
	    }

	    WaypointMsg msg = waypointPub.newMessage();
	    msg.setX(rand.nextDouble()*10);
	    msg.setY(rand.nextDouble()*10);
	    msg.setTheta(-1);
	    waypointPub.publish(msg);
	    }*/
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
	
	posTargMsgPub = node.newPublisher("/state/PositionTarget", "rss_msgs/PositionTargetMsg");
	ctrlStatePub = node.newPublisher("/state/State", std_msgs.String._TYPE);
	waypointPub = node.newPublisher("/state/Waypoint", "rss_msgs/WaypointMsg");

	velPub = node.newPublisher("/state/Velocity", "rss_msgs/VelocityMsg");
        initPub = node.newPublisher("/state/Initialized", "rss_msgs/InitializedMsg");
	//rand = new Random();

	ballLocationPub = node.newPublisher("/state/BallLocation", "rss_msgs/BallLocationMsg");


	//yay hacks to see if things work
	
	//	motorsPub = node.newPublisher("command/Motors", "rss_msgs/PositionTargetMsg");
	
	//end hacks

        posSub = node.newSubscriber("/loc/Position", "rss_msgs/PositionMsg");
        posSub.addMessageListener(new MessageListener<rss_msgs.PositionMsg>() {
            @Override
	    public void onNewMessage(rss_msgs.PositionMsg message) {
                state.handle(message);
            }
        });
    
    
    
        /*waypointSub = node.newSubscriber("/path/Waypoint", "rss_msgs/WaypointMsg");
        waypointSub.addMessageListener(new MessageListener<rss_msgs.WaypointMsg>(){
            @Override
            public void onNewMessage(rss_msgs.WaypointMsg message){
                handle(message);
                System.out.println("State Machine got a waypoint");
        }
        });*/
        
        
        bumpSub = node.newSubscriber("/sense/Bump", "rss_msgs/BumpMsg");
        bumpSub.addMessageListener(new MessageListener<rss_msgs.BumpMsg>(){
            @Override
            public void onNewMessage(rss_msgs.BumpMsg message){
                state.handle(message);
                //System.out.println("State Machine got a bump");
                
            }
        });
        
        breakbeamSub = node.newSubscriber("/sense/BreakBeam", "rss_msgs/BreakBeamMsg");
        breakbeamSub.addMessageListener(new MessageListener<rss_msgs.BreakBeamMsg>() {
            @Override
            public void onNewMessage(rss_msgs.BreakBeamMsg message){
                state.handle(message);
                //System.out.println("State Machine got a broken beam");
            }
        });
        
        sonarSub = node.newSubscriber("/sense/Sonar", "rss_msgs/SonarMsg");
        sonarSub.addMessageListener(new MessageListener<rss_msgs.SonarMsg>(){
            @Override
            public void onNewMessage(rss_msgs.SonarMsg message){
                state.handle(message);
                //System.out.println("State Machine got a sonar (or a few)");
            }
        });



	state = startState;
        //count = 0; // for testing --bhomberg

        
        ballLocationSub = node.newSubscriber("/vis/BallLocation", "rss_msgs/BallLocationMsg");
        ballLocationSub.addMessageListener(new MessageListener<rss_msgs.BallLocationMsg>() {
            @Override
            public void onNewMessage(rss_msgs.BallLocationMsg message){
                state.handle(message);
                System.out.println("State Machine got a ball location message");
            }
        });
        
        mapSub = node.newSubscriber("/loc/Map", "rss_msgs/MapMsg");
        mapSub.addMessageListener(new MessageListener<MapMsg>() {
            @Override
            public void onNewMessage(MapMsg msg) {
                handleMapMsg(msg);
            }
        });
        
    }
    
        

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("rss/statemachine");
    }
}
