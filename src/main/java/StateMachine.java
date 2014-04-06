package StateMachine;

//import java.util.Arrays;
//import java.util.concurrent.ArrayBlockingQueue;

import org.ros.message.MessageListener;
import rss_msgs.PositionMsg;
import rss_msgs.WaypointMsg;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

/**
 * 
 * @author bhomberg
 * 
 */
public class StateMachine extends AbstractNodeMain implements Runnable {

    private VisionGUI gui;

    protected boolean firstUpdate = true;

    public Subscriber<PositionMsg> posSub;
    
    public Publisher<WaypointMsg> waypointPub;
    
    private WaypointMsg currGoal;
    
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
            WaypointMsg msg = waypointPub.newMessage();
            msg.setX() = Math.random()*5.0;
            msg.setY() = Math.random()*5.0;
            waypointPub.publish(msg);
            currGoal = msg;
        }
    }
    
    public double dist(PositionMsg odo){
        return Math.sqrt(Math.pow(odo.getX() - currGoal.getX(), 2) + 
                            Math.pow(odo.getY() - currGoal.getY(), 2));
        
    }

    @Override
    public void run() {
        while (true) {
	    System.out.println("In SM run loop");
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
   
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("rss/statemachine");
    }
}
