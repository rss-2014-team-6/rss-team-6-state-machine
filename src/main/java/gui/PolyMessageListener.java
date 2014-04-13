package gui;

import org.ros.message.MessageListener;
import gui_msgs.GUIPolyMsg;

import java.awt.Color;
import java.awt.geom.*;
import java.util.ArrayList;
import java.util.List;

public class PolyMessageListener implements MessageListener<GUIPolyMsg> {

	private MapGUI gui;

	public PolyMessageListener(MapGUI mapGUI) {
		gui = mapGUI;
	}

	@Override
	public synchronized void onNewMessage(GUIPolyMsg msg) {
		List<Point2D.Double> vertices = new ArrayList<Point2D.Double>();
		for (int i = 0; i < msg.getNumVertices(); i++){
                    Point2D.Double p = new Point2D.Double(msg.getX()[i], msg.getY()[i]);
                    vertices.add(p);
		}
		boolean closed = msg.getClosed() == 1;
		boolean filled = msg.getFilled() == 1;
		Color c = new Color((int)msg.getC().getR(),
                                    (int)msg.getC().getG(),
                                    (int)msg.getC().getB());
		gui.addPoly(vertices, closed, filled, c);
	}

}
