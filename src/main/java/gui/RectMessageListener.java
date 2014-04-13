package gui;

import java.awt.Color;

import org.ros.message.MessageListener;
import gui_msgs.ColorMsg;
import gui_msgs.GUIRectMsg;

public class RectMessageListener implements MessageListener<GUIRectMsg> {

	private MapGUI gui;

	public RectMessageListener(MapGUI mapGUI) {
		this.gui = mapGUI;
	}

	@Override
	public void onNewMessage(GUIRectMsg message) {
            boolean filled = message.getFilled() == 1;
            Color color = getColorFromMsg(message.getC());
            gui.addRect(message.getX(), message.getY(),
                        message.getWidth(), message.getHeight(),
                        filled, color);
	}

	public Color getColorFromMsg(ColorMsg c) {
		Color color;
		if (c == null){
                    color = gui.rectColor;
		}else {
                    color = new Color((int)c.getR(), (int)c.getG(), (int)c.getB());
		}
		return color;
	}

}
