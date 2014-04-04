package StateMachine;

//import java.util.Arrays;
//import java.util.concurrent.ArrayBlockingQueue;

import org.ros.message.MessageListener;
import rss_msgs.MotionMsg;
//import rss_msgs.BallLocationMsg;
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

    public Subscriber<rss_msgs.PositionMsg> posSub;

    /**
     * <p>
     * Create a new StateMachine object.
     * </p>
     */
    public StateMachine() {

        setInitialParams();

        gui = new VisionGUI();
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
