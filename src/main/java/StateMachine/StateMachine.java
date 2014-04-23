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
    
    public Publisher<PositionTargetMsg> posTargMsgPub;
    public Publisher<std_msgs.String> ctrlStatePub;
    public Publisher<VelocityMsg> velPub;
    
    
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
	
    
    
    private Random rand; // for testing --bhomberg
    
    private int count;
   
    public double dist(PositionMsg odo){
	// not sure how you want to deal with this more nicely, this is hackish --bhomberg
	if(currGoal == null)
	    return Double.POSITIVE_INFINITY;
        return Math.sqrt(Math.pow(odo.getX() - currGoal.getX(), 2) + 
                            Math.pow(odo.getY() - currGoal.getY(), 2));
        
    }

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
     * Get time since start of execution in ms.
     */
    public long getTime(){
        return System.currentTimeMillis() - startTime;
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
    }
    
        

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("rss/statemachine");
    }
}
