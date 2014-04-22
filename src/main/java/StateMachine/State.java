package StateMachine;


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


    public State handle (BumpMsg msg){

	// commenting all these out since they'll clog up the debug screen
	//System.out.println("Node " + name + " handled bump msg");
	return this;
    }


    public State handle (SonarMsg msg){

	//System.out.println("Node " + name + " handled sonar msg");
	return this;
    }



    public State handle (PositionMsg msg){

	//System.out.println("Node " + name + " handled pose msg");
	return this;
    }


    public State handle (WaypointMsg msg){

	//System.out.println("Node " + name + " handled waypoint msg");
	return this;
    }


    public State handle (OdometryMsg msg){

	//System.out.println("Node " + name + " handled odo msg");
	return this;
    }


    public State handle (BreakBeamMsg msg){

	//System.out.println("Node " + name + " handled breakbeam msg");
	return this;
    }

    public State handle (PositionTargetMsg msg){

	//System.out.println("Node " + name + " handled postarg msg");
	return this;
    }
}
