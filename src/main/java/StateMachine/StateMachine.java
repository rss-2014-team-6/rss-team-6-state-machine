package StateMachine;

//import java.util.Arrays;
//import java.util.concurrent.ArrayBlockingQueue;

import org.ros.message.MessageListener;
import rss_msgs.PositionMsg;
import rss_msgs.VelocityMsg;
import rss_msgs.PositionTargetMsg;
import rss_msgs.WaypointMsg;
import rss_msgs.BumpMsg;
import rss_msgs.BreakBeamMsg;
import rss_msgs.SonarMsg;
import rss_msgs.BallLocationMsg;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import java.util.Random;
import java.lang.InterruptedException;

/**
 * 
 * @author cwetters
 * 
 */
public class StateMachine extends AbstractNodeMain implements Runnable {

    public Subscriber<PositionMsg> posSub;
    //public Subscriber<WaypointMsg> waypointSub;
    public Subscriber<BumpMsg> bumpSub;
    public Subscriber<BreakBeamMsg> breakbeamSub;
    public Subscriber<SonarMsg> sonarSub;
    public Subscriber<BallLocationMsg> ballLocationSub;
    
    public Publisher<PositionTargetMsg> posTargMsgPub;
    public Publisher<std_msgs.String> ctrlStatePub;

    public Publisher<VelocityMsg> velPub;

    public Publisher<BallLocationMsg> ballLocationPub;

    
    
    //Probably should be changed to Waypoint..
    public Publisher<PositionTargetMsg> motorsPub;

    //temporarily output waypoints so we can see if the motors move --bhomberg
    public Publisher<WaypointMsg> waypointPub;
    //end hacky
    
    
    private PositionTargetMsg currGoal;
    private WaypointMsg currWaypoint;
    
    private final double ACCEPTABLE_ERROR = 0.05;
    
    private long startTime;
    
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

    private State currState;
    

    /*private State state = new State ("example"){
	    @Override
		public void handle (PositionMsg msg){
		System.out.println("I'm an overwritten state handler for pose");
	    }
	    };*/

    private State startState = new State("start"){
	    @Override public State handle (BumpMsg msg){
		VelocityMsg vmsg = velPub.newMessage();
		vmsg.setTranslationVelocity(0);
		vmsg.setRotationVelocity(0);
		velPub.publish(vmsg);
		System.out.println("curr time: " + getTime());
		if(getTime() >= 10000) // wait 10 seconds
		    return spinState;
		return this;
	    }
	};
    
    private State spinState = new State ("spin"){
	    @Override public State handle(BumpMsg msg){
		if(getTime() <= 40000){ // spin for 30 s
		    VelocityMsg vmsg = velPub.newMessage();
		    vmsg.setTranslationVelocity(0);
		    vmsg.setRotationVelocity(2.0);
		    velPub.publish(vmsg);
		    return this;
		}
		else{
		    VelocityMsg vmsg = velPub.newMessage();
		    vmsg.setTranslationVelocity(0);
		    vmsg.setRotationVelocity(0);
		    velPub.publish(vmsg);
                    System.out.println("Transition to goal state");
		    return goalState;
		}

	    }
	};

    private State goalState = new State("goal"){
            @Override public State handle(BumpMsg msg){
                System.out.println("Goal state");
                PositionTargetMsg posTargMsg = posTargMsgPub.newMessage();
                posTargMsg.setX(-0.1);
                posTargMsg.setY(3.6);
                posTargMsg.setTheta(-1);
                posTargMsgPub.publish(posTargMsg);
                currGoal = posTargMsg;
                return this;
            }
            @Override public State handle(PositionMsg msg){
                final double GOAL_THRESH = 0.03;
                if (dist(msg) < GOAL_THRESH) {
                    System.out.println("Reached goal!");
                    return stopState;
                }
                return this;
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
                lastState = lost;
            }
        }
    };
    
    private State spin = new State("spin"){
      int spins = -1;
      double startTheta = 0;
      final double SPIN_THRESHOLD = 0.5;
      @Override
      public void handle(PositionMsg msg){ 
          if (spins == -1){
              startTheta = msg.getTheta();
          }
          if ((msg.getTheta - startTheta)%360 < SPIN_THRESHOLD){ //probs a better way to do this
              spins++;   
          }
          if (spins > 2){
              state = wander;
              lastState = spin; 
              publishWander();
          }
      }
      
      @Override
      public void handle(BallLocationMsg msg){
          state = visualServo;
          lastState = spin;
                  
      }
    };
    
    private State visualServo = new State("visualServo"){
       @Override
       public void handle(BallLocationMsg msg){
           std_msgs.String ctrlState = ctrlStatePub.newMessage();
           ctrlState.setData("visualServo");
           ctrlStatePub.publish(ctrlState);
           
           BallLocationMsg ballMsg = ballLocationPub.newMessage();
           
           ballMsg.setRange(0);
           ballMsg.setBearing(0);
           ballLocatoinPub.publish(ballMsg);
           
           //check if ball is no longer in frame, drive forward extra x feet, return to previous state
           
       }
    };
    
    private State wander = new State("wander"){
        @Override
        final double WANDER_THRESHOLD = 0.5;
        public void handle(PositionMsg msg){
            if (timeToGoHome()){
                state = driveBuildSite;
                lastState = driveLost;
                PositionTargetMsg targmsg = posTargMsgPub.newMessage();
                targmsg.setX(0);
                targmsg.setY(0);
                targmsg.setTheta(0);
                currGoal = targmsg;
                posTargMsgPub.publish(targmsg);
                return;
            }
            
            if(Math.sqrt(Math.pow(msg.getX() - currGoal.getX(), 2) + Math.pow(msg.getY() - currGoal.getY(), 2))< WANDER_THRESHOLD){
                publishWander();
            }
        }
        
        public void handle(BallLocationMsg msg){
            state = visualServo;
            lastState = wander;
        }
    };
    
    private void publishWander(){
        PositionTargetMsg msg = posTargMsgPub.newMessage();
        msg.setX(Math.random()*3 - .5);
        msg.setY(Math.random()*4.5 - .5);
        currGoal = msg;
        posTargMsgPub.publish(msg);    
    }
    
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
                lastState = driveLost;
            }
        }
    };
    
    private State driveBuildSite = new State("driveBuildSite"){
        @Override
        final double DISTANCE_THRESHOLD = 0.5;
        final double HOME_X = 0.0;
        final double HOME_Y = 0.0;
        public void handle(PositionMsg msg){
            if (Math.sqrt(Math.pow(HOME_X - msg.getX(), 2) + Math.pow(HOME_Y - msg.getY(),2)) < DISTANCE_THRESHOLD){
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

    
///////////////////////////////////////////////////////////////////////////////////////////////end States
    
    
    private Random rand; // for testing --bhomberg
    
    private int count;
   
////////////////////////////////////////////////////////////////////////////////handlers

    public void handle(PositionMsg odo){
        //IMPLEMENT_STATES
        //state.handle(odo);
       
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
	waypointPub = node.newPublisher("/path/Waypoint", "rss_msgs/WaypointMsg");

	velPub = node.newPublisher("/state/Velocity", "rss_msgs/VelocityMsg");
	//rand = new Random();

	ballLocationPub = node.newPublisher("/state/BallLocation", "rss_msgs/BallLocationMsg");


	//yay hacks to see if things work
	
	//	motorsPub = node.newPublisher("command/Motors", "rss_msgs/PositionTargetMsg");
	
	//end hacks

        posSub = node.newSubscriber("/loc/Position", "rss_msgs/PositionMsg");
        posSub.addMessageListener(new MessageListener<rss_msgs.PositionMsg>() {
            @Override
	    public void onNewMessage(rss_msgs.PositionMsg message) {
                currState = currState.handle(message);
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
                currState = currState.handle(message);
                //System.out.println("State Machine got a bump");
                
            }
        });
        
        breakbeamSub = node.newSubscriber("/sense/BreakBeam", "rss_msgs/BreakBeamMsg");
        breakbeamSub.addMessageListener(new MessageListener<rss_msgs.BreakBeamMsg>() {
            @Override
            public void onNewMessage(rss_msgs.BreakBeamMsg message){
                currState = currState.handle(message);
                //System.out.println("State Machine got a broken beam");
            }
        });
        
        sonarSub = node.newSubscriber("/sense/Sonar", "rss_msgs/SonarMsg");
        sonarSub.addMessageListener(new MessageListener<rss_msgs.SonarMsg>(){
            @Override
            public void onNewMessage(rss_msgs.SonarMsg message){
                currState = currState.handle(message);
                //System.out.println("State Machine got a sonar (or a few)");
            }
        });



	currState = startState;
        //count = 0; // for testing --bhomberg

        
        ballLocationSub = node.newSubscriber("/vis/BallLocation", "rss_msgs/BallLocationMsg");
        ballLocationSub.addMessageListener(new MessageListener<rss_msgs.BallLocationMsg>() {
            @Override
            public void onNewMessage(rss_msgs.BallLocationMsg message){
                handle(message);
                System.out.println("State Machine got a ball location message");
            }
        });
        
    }
    
        

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("rss/statemachine");
    }
}
