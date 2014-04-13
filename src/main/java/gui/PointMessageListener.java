package gui;

import java.awt.Color;

import org.ros.message.MessageListener;
import gui_msgs.GUIPointMsg;

public class PointMessageListener implements MessageListener<GUIPointMsg> {

    private SonarGUI gui;

    public PointMessageListener(SonarGUI sonarGUI) {
        this.gui = sonarGUI;
    }

    @Override
	public void onNewMessage(GUIPointMsg msg) {
        int r = (int) msg.getColor().getR();
        int g = (int) msg.getColor().getG();
        int b = (int) msg.getColor().getB();

        if (r<0 || g < 0 || b < 0){
            gui.addPoint(msg.getX(), msg.getY(), (int) msg.getShape());
        } else{
            Color c = new Color(r, g, b);
            gui.addPoint(msg.getX(), msg.getY(), (int) msg.getShape(), c);
        }
    }

}
