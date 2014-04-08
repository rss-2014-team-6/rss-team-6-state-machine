package StateMachine;

//import java.util.Arrays;
//import java.util.concurrent.ArrayBlockingQueue;

import org.ros.message.MessageListener;
import rss_msgs.PositionMsg;
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

/**
 * 
 * @author cwetters
 * 
 */
public class StateMachine extends AbstractNodeMain implements Runnable {

    private VisionGUI gui;

    protected boolean firstUpdate = true;

    public Subscriber<PositionMsg> posSub;
    public Subscriber<WaypointMsg> waypointSub;
    public Subscriber<BumpMsg> bumpSub;
    public Subscriber<BreakBeamMsg> breakbeamSub;
    public Subscriber<SonarMsg> sonarSub;
    
    public Publisher<PositionTargetMsg> posTargMsgPub;
    public Publisher<std_msgs.String> ctrlStatePub;
    
    
    //Probably should be changed to Waypoint..
    public Publisher<PositionTargetMsg> motorsPub;
    //end hacky
    
    
    private PositionTargetMsg currGoal;
    private WaypointMsg currWaypoint;
    
    private final double ACCEPTABLE_ERROR = 0.05;
    
    private int age;
    private int state;
    
    private final int ERROR = -1;
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
    private final int DRIVE_BUILD = 34; //drive to next place to build a tower
    
    
   

    /**
     * <p>
     * Create a new StateMachine object.
     * </p>
     */
    public StateMachine() {

        gui = new VisionGUI();
    }

    public void handle(PositionMsg odo){
       
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
        **/
        //end hacks
    }
    
    public void handle(BumpMsg bump){
        System.out.println("bump message handled");
    }
    public void handle(BreakBeamMsg bbeam){
        System.out.println("bbeam handled");
    }
    public void handle(SonarMsg sonar){
        System.out.println("sonar handled");
    }
    public double dist(PositionMsg odo){
        return Math.sqrt(Math.pow(odo.getX() - currGoal.getX(), 2) + 
                            Math.pow(odo.getY() - currGoal.getY(), 2));
        
    }

    @Override
    public void run() {
        while (true) {
	    System.out.println("In SM run loop");
	    //this used to do visiony things, idk what it's supposed to do now. 
	    
	}
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
	
	posTargMsgPub = node.newPublisher("/state/PositionTarget", "rss_msgs/PositionTargetMsg");
	ctrlStatePub = node.newPublisher("/state/State", std_msgs.String._TYPE);
	
	//yay hacks to see if things work
	/**
	motorsPub = node.newPublisher("command/Motors", "rss_msgs/PositionTargetMsg");
	*/
	//end hacks

        posSub = node.newSubscriber("/loc/Position", "rss_msgs/PositionMsg");
        posSub.addMessageListener(new MessageListener<rss_msgs.PositionMsg>() {
            @Override
	    public void onNewMessage(rss_msgs.PositionMsg message) {
                handle(message);
                if (firstUpdate) {
                    firstUpdate = false;
                    gui.resetWorldToView(message.getX(), message.getY());
                }
               
                gui.setRobotPose(message.getX(), message.getY(), message.getTheta());

		System.out.println("State Machine got position message!");
            }
        });
    
    
    
        waypointSub = node.newSubscriber("/path/Waypoint", "rss_msgs/WaypointMsg");
        waypointSub.addMessageListener(new MessageListener<rss_msgs.WaypointMsg>(){
            @Override
            public void onNewMessage(rss_msgs.WaypointMsg message){
                handle(message);
                System.out.println("State Machine got a waypoint");
        }
        });
        
        
        bumpSub = node.newSubscriber("/sense/Bump", "rss_msgs/BumpMsg");
        bumpSub.addMessageListener(new MessageListener<rss_msgs.BumpMsg>(){
            @Override
            public void onNewMessage(rss_msgs.BumpMsg message){
                handle(message);
                System.out.println("State Machine got a bump");
                
            }
        });
        
        breakbeamSub = node.newSubscriber("/sense/BreakBeam", "rss_msgs/BreakBeamMsg");
        breakbeamSub.addMessageListener(new MessageListener<rss_msgs.BreakBeamMsg>() {
            @Override
            public void onNewMessage(rss_msgs.BreakBeamMsg message){
                handle(message);
                System.out.println("State Machine got a broken beam");
            }
        });
        
        sonarSub = node.newSubscriber("/sense/Sonar", "rss_msgs/SonarMsg");
        sonarSub.addMessageListener(new MessageListener<rss_msgs.SonarMsg>(){
            @Override
            public void onNewMessage(rss_msgs.SonarMsg message){
                handle(message);
                System.out.println("State Machine got a sonar (or a few)");
            }
        });
        
        
    }
    
        

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("rss/statemachine");
    }
}
