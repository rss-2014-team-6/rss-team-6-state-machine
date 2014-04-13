package gui;

import java.awt.Color;

import org.ros.message.MessageListener;
import gui_msgs.GUISegmentMsg;

public class SegmentMessageListener implements MessageListener<GUISegmentMsg> {

	private SonarGUI gui;

	public SegmentMessageListener(SonarGUI sonarGUI) {
		this.gui = sonarGUI;
	}

	@Override
	public void onNewMessage(GUISegmentMsg msg) {
            int r = (int) msg.getColor().getR();
            int g = (int) msg.getColor().getG();
            int b = (int) msg.getColor().getB();
            if (r<0 || g < 0 || b < 0){
                gui.addSegment(msg.getStartX(), msg.getStartY(),
                               msg.getEndX(), msg.getEndY());
            } else{
                Color c = new Color(r, g, b);
                gui.addSegment(msg.getStartX(), msg.getStartY(),
                               msg.getEndX(), msg.getEndY(), c);
            }
	}

}
