package StateMachine;

//import java.util.Arrays;
//import java.util.concurrent.ArrayBlockingQueue;

import org.ros.message.MessageListener;
import rss_msgs.PositionMsg;
import rss_msgs.PositionTargetMsg;
import rss_msgs.WaypointMsg;
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
    
    public Publisher<PositionTargetMsg> posTargMsgPub;
    
    
    //Probably should be changed to Waypoint..
    public Publisher<PositionTargetMsg> motorsPub;
    //end hacky
    
    
    private PositionTargetMsg currGoal;
    private WaypointMsg currWaypoint;
    
    private final double ACCEPTABLE_ERROR = 0.05;
    
   

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
        PositionTargetMsg msg = motorsPub.newMessage();
        msg.setX(way.getX());
        msg.setY(way.getY());
        msg.setTheta(way.getTheta());
        motorsPub.publish(msg);
        //end hacks
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
	
	//yay hacks to see if things work
	motorsPub = node.newPublisher("command/Motors", "rss_msgs/PositionTargetMsg");
	//end hacks

        posSub = node.newSubscriber("/loc/position", "rss_msgs/PositionMsg");
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
        waypointSub.addMessageListener(new MessageListener<rss_msgs.WaypointMsg()>{
            @Override
            public void onNewMessage(rss_msgs.WaypointMsg message){
                handle(message);
                System.out.println("State Machine got a waypoint");
        }
        });
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("rss/statemachine");
    }
}
