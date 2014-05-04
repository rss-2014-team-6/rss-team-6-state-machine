package StateMachine;


import rss_msgs.BallLocationMsg;
import rss_msgs.PositionMsg;
import rss_msgs.PositionTargetMsg;
import rss_msgs.WaypointMsg;
import rss_msgs.BumpMsg;
import rss_msgs.BreakBeamMsg;
import rss_msgs.SonarMsg;
import rss_msgs.OdometryMsg;

public class State {

    private String name;

    public State (String name){
	this.name = name;
    }

    public String getName(){
	return name;
    }
    
    public void handle (BumpMsg msg){

	// commenting all these out since they'll clog up the debug screen
	//System.out.println("Node " + name + " handled bump msg");
    }


    public void handle (SonarMsg msg){

	//System.out.println("Node " + name + " handled sonar msg");
    }



    public void handle (PositionMsg msg){

	//System.out.println("Node " + name + " handled pose msg");
    }


    public void handle (WaypointMsg msg){

	//System.out.println("Node " + name + " handled waypoint msg");
    }


    public void handle (OdometryMsg msg){

	//System.out.println("Node " + name + " handled odo msg");
    }


    public void handle (BreakBeamMsg msg){

	//System.out.println("Node " + name + " handled breakbeam msg");
    }

    public void handle (PositionTargetMsg msg){

	//System.out.println("Node " + name + " handled postarg msg");
    }

    public void handle (BallLocationMsg msg){
    }
}
