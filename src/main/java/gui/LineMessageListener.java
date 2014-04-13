package gui;

import java.awt.Color;

import org.ros.message.MessageListener;
import gui_msgs.GUILineMsg;

public class LineMessageListener implements MessageListener<GUILineMsg> {

	private SonarGUI gui;

	public LineMessageListener(SonarGUI sonarGUI) {
		this.gui = sonarGUI;
	}

	@Override
	public void onNewMessage(GUILineMsg msg) {
            int r = (int) msg.getColor().getR();
            int g = (int) msg.getColor().getG();
            int b = (int) msg.getColor().getB();
            if (r<0 || g < 0 || b < 0) {
                gui.setLine(msg.getLineA(), msg.getLineB(), msg.getLineC());
            } else{
                Color c = new Color(r, g, b);
                gui.setLine(msg.getLineA(), msg.getLineB(), msg.getLineC(), c);
            }
	}

}
