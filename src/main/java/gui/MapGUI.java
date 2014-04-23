package gui;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.GeneralPath;
import java.awt.geom.PathIterator;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

import org.ros.message.MessageListener;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;

import gui_msgs.GUIParticleCloudMsg;
import gui_msgs.GUIPathMsg;
import gui_msgs.GUIGraphMsg;
import gui_msgs.PointDataMsg;
import gui_msgs.PointMappingMsg;


/**
 * <p>Extends <code>LocalNavigation.SonarGUI</code> to display map-related
 * data (first read the doc for that class).</p>
 *
 * <p>New methods (and corresponding ROS messages) have been added to draw
 * rectangles, polygons, paths, and (RRT) graphs.</p>
 * 
 * @author vona
 **/
@SuppressWarnings("serial")
public class MapGUI extends SonarGUI implements NodeMain{

    /**
     * <p>The application name.</p>
     **/
    public static final String APPNAME = "MapGUI";

    /**
     * <p>Bitfield constant for {@link GUIEraseMessage}.</p>
     **/
    public static final int ERASE_RECTS = 1<<6;

    /**
     * <p>Bitfield constant for {@link GUIEraseMessage}.</p>
     **/
    public static final int ERASE_POLYS = 1<<7;

    /**
     * <p>Line width of the lines making up a {@link MapGUI.Rect} in
     * pixels.</p>
     **/
    public static final float RECT_LINE_WIDTH = 1.5f;

    /**
     * <p>Line width of the lines making up a {@link MapGUI.Poly} in
     * pixels.</p>
     **/
    public static final float POLY_LINE_WIDTH = 1.5f;

    /**
     * <p>Line width of the lines making up a {@link MapGUI.Path} in
     * pixels.</p>
     **/
    public static final float PATH_LINE_WIDTH = 2.0f;

    /**
     * <p>Line width of the lines making up a {@link MapGUI.Graph} in
     * pixels.</p>
     **/
    public static final float GRAPH_LINE_WIDTH = 1.0f;

    /**
     * <p>Default color for {@link MapGUI.Rect}s.</p>
     **/
    public static final Color DEFAULT_RECT_COLOR = Color.RED;

    /**
     * <p>Default color for {@link MapGUI.Poly}s.</p>
     **/
    public static final Color DEFAULT_POLY_COLOR = Color.RED;

    /**
     * <p>Default color for {@link MapGUI.Path}s.</p>
     **/
    public static final Color DEFAULT_PATH_COLOR = Color.RED;

    /**
     * <p>Default color for {@link MapGUI.Graph}s.</p>
     **/
    public static final Color DEFAULT_GRAPH_COLOR = Color.GRAY;

    /**
     * <p>Default color for {@link MapGUI.ParticleCloud}s.</p>
     **/
    public static final Color DEFAULT_PARTICLE_CLOUD_COLOR = Color.GRAY;

    /**
     * <p>Whether to paint the rects.</p>
     **/
    protected boolean rectsEnabled = true;

    /**
     * <p>Whether to paint the polys.</p>
     **/
    protected boolean polysEnabled = true;

    /**
     * <p>The current {@link MapGUI.Rect} color.</p>
     **/
    protected Color rectColor = dupColor(DEFAULT_RECT_COLOR);

    /**
     * <p>The current {@link MapGUI.Poly} color.</p>
     **/
    protected Color polyColor = dupColor(DEFAULT_POLY_COLOR);

    /**
     * <p>A visual rectangle.</p>
     **/
    protected class Rect extends Glyph {

        /**
         * <p>Rect color.</p>
         **/
        Color color;

        /**
         * <p>Whether to draw filled.</p>
         **/
        boolean filled;

        /**
         * <p>The rect in world coords.</p>
         **/
        Rectangle2D.Double r = new Rectangle2D.Double();

        /**
         * <p>Create a new rect.</p>
         **/
        Rect(Rectangle2D.Double r, boolean filled, Color color) {
            this.color = dupColor(color);
            this.filled = filled;
            this.r.setRect(r);
        }

        /**
         * <p>Create a new rect.</p>
         **/
        Rect(double x, double y, double width, double height,
             boolean filled, Color color) {

            this.color = dupColor(color);
            this.filled = filled;

            this.r.x = x;
            this.r.y = y;
            this.r.width = width;
            this.r.height = height;
        }

        @Override public int hashCode() {
            return 0;
        }

        @Override public boolean equals(Object other) {
            Rect another = (Rect)other;
            return ( color.equals(another.color) &&
                     filled == another.filled &&
                     r.equals(another.r) );
        }

        /**
         * <p>Paints the rect.</p>
         *
         * <p>Assumes line width is already set.</p>
         *
         * @param g2d the graphics context
         **/
        @Override public void paint(Graphics2D g2d) {

            g2d.setColor(color);

            if (filled)
                g2d.fill(r);

            g2d.draw(r);
        }
    }

    /**
     * <p>A visual polygon.</p>
     **/
    protected class Poly extends Glyph {

        /**
         * <p>Poly color.</p>
         **/
        Color color;

        /**
         * <p>Whether to close the poly.</p>
         **/
        boolean closed;

        /**
         * <p>Whether to draw filled.</p>
         **/
        boolean filled;

        /**
         * <p>The poly in world coords.</p>
         **/
        GeneralPath path = new GeneralPath();

        /**
         * <p>Create a new poly.</p>
         **/
        Poly(java.util.List<Point2D.Double> vertices,
             boolean closed, boolean filled, Color color) {

            this.color = dupColor(color);
            this.closed = closed;
            this.filled = filled;

            boolean first = true;
            for (Point2D.Double vertex : vertices) {

                float x = (float) (vertex.getX());
                float y = (float) (vertex.getY());

                if (first) {
                    path.moveTo(x, y);
                    first = false;
                } else {
                    path.lineTo(x, y);
                }
            }

            if (closed)
                path.closePath();
        }

        @Override public int hashCode() {
            return 0;
        }

        @Override public boolean equals(Object other) {
            Poly another = (Poly)other;
            PathIterator pi1 = path.getPathIterator(null);
            PathIterator pi2 = another.path.getPathIterator(null);
            while ( !pi1.isDone() && !pi2.isDone() ) {
                double[] c1 = new double[6];
                int t1 = pi1.currentSegment(c1);
                double[] c2 = new double[6];
                int t2 = pi2.currentSegment(c2);
                if ( t1 != t2 ) return false;
                if ( c1[0] != c2[0] || c1[1] != c2[1] ) return false;
                pi1.next();
                pi2.next();
            }
            if ( !(pi1.isDone() && pi2.isDone()) ) return false;

            return ( color.equals(another.color) &&
                     closed == another.closed &&
                     filled == another.filled );
        }

        /**
         * <p>Paints the poly.</p>
         *
         * <p>Assumes line width is already set.</p>
         *
         * @param g2d the graphics context
         **/
        @Override public void paint(Graphics2D g2d) {

            g2d.setColor(color);

            if (filled)
                g2d.fill(path);

            g2d.draw(path);
        }
    }

    /**
     * <p>A visual path of 2D points.</p>
     **/
    protected class Path extends Glyph {
        /**
         * <p>Path color.</p>
         **/
        Color color = DEFAULT_PATH_COLOR;

        /**
         * <p>The path in world coords.</p>
         **/
        GeneralPath path = new GeneralPath();

        /**
         * <p>Constructor, making an initially empty path.</p>
         **/
        Path() {}

        /**
         * <p>Update the path.</p>
         *
         * @param pathList list of points in the linear path
         **/
        public void updatePath(List<Point2D.Double> pathList) {
            path.reset();
            if (pathList.size() > 0) {
                Point2D.Double first = pathList.get(0);
                path.moveTo(first.getX(), first.getY());
            }
            for (int i = 1; i < pathList.size(); i++) {
                Point2D.Double next = pathList.get(i);
                path.lineTo(next.getX(), next.getY());
            }
        }

        /**
         * <p>Set the color.</p>
         *
         * @param c color to set
         **/
        public void setColor(Color c) {
            color = dupColor(c);
        }

        /**
         * <p>Paints the path.</p>
         *
         * <p>Assumes line width is already set.</p>
         *
         * @param g2d the graphics context
         **/
        @Override public void paint(Graphics2D g2d) {
            g2d.setColor(color);

            g2d.draw(path);
        }
    }

    /**
     * <p>A visual graph of interconnected 2D point.</p>
     **/
    protected class Graph extends Glyph {
        /**
         * <p>Graph color.</p>
         **/
        Color color = DEFAULT_GRAPH_COLOR;

        /**
         * <p>The GeneralPath representing the visual graph in world coords.</p>
         **/
        GeneralPath path = new GeneralPath();

        /**
         * <p>Constructor, making an initially empty graph.</p>
         **/
        Graph() {}

        /**
         * <p>Update the graph.</p>
         *
         * @param graphMap map from 2D points to list of other points
         **/
        public void updateGraph(Map<Point2D.Double, ArrayList<Point2D.Double>> graphMap) {
            path.reset();
            for (Point2D.Double first : graphMap.keySet()) {
                for (Point2D.Double second : graphMap.get(first)) {
                    path.moveTo(first.getX(), first.getY());
                    path.lineTo(second.getX(), second.getY());
                }
            }
        }

        /**
         * <p>Set the color.</p>
         *
         * @param c color to set
         **/
        public void setColor(Color c) {
            color = dupColor(c);
        }

        /**
         * <p>Paints the graph.</p>
         *
         * <p>Assumes line width is already set.</p>
         *
         * @param g2d the graphics context
         **/
        @Override public void paint(Graphics2D g2d) {
            g2d.setColor(color);

            g2d.draw(path);
        }
    }

    /**
     * <p>A cloud of weighted particles.</p>
     **/
    protected class ParticleCloud extends Glyph {
        /**
         * <p>List of GUIPoint Glyphs defining this point cloud.
         **/
        List<GUIPoint> guiPoints = new ArrayList<GUIPoint>();

        /**
         * <p>Constructor, making an initially empty cloud.</p>
         **/
        ParticleCloud() {}

        /**
         * <p>Update the particle cloud.</p>
         *
         * @param points points in the cloud
         * @param weights negative log of weights for each point
         **/
        public void updateParticleCloud(List<Point2D.Double> points, double[] weights) {
            guiPoints.clear();
            if (points.size() != weights.length) {
                throw new RuntimeException("points and weights lengths must match!");
            }
            /*
            double maxConvertedWeight = 0.0;
            for (int i = 0; i < weights.length; i++) {
                double converted = Math.exp(-1 * weights[i]);
                if (converted > maxConvertedWeight) {
                    maxConvertedWeight = converted;
                }
            }
            */
            double maxWeight = 0.0;
            for (int i = 0; i < weights.length; i++) {
                if (weights[i] > maxWeight) {
                    maxWeight = weights[i];
                }
            }
            
            for (int i = 0; i < points.size(); i++) {
                Point2D.Double pt = points.get(i);
                double weight = weights[i];
                //double red = Math.exp(-1 * weight) / maxConvertedWeight; // scaled
                double red = (maxWeight - weight) / maxWeight;
                System.out.println("Point: " + pt + ", color: " + red + ", weight: " + weight);
                // Color the point based on weight
                GUIPoint guiPt = new GUIPoint(pt.x, pt.y, O_POINT, new Color((float)red, 0.0f, 0.0f));
                guiPoints.add(guiPt);
            }
        }

        /**
         * <p>Paints the cloud.</p>
         *
         * @param g2d the graphics context
         **/
        @Override public void paint(Graphics2D g2d) {
            setLineWidth(g2d, POINT_LINE_WIDTH);

            for (GUIPoint guiPt : guiPoints) {
                System.out.println("Paint point: " + guiPt + ", color: " + guiPt.color);
                guiPt.paint(g2d);
            }
        }
    }

    /**
     * <p>All the {@link MapGUI.Rect}s.</p>
     **/
    protected java.util.Set<Rect> rects =
        Collections.synchronizedSet(new HashSet<Rect>());

    /**
     * <p>All the {@link MapGUI.Poly}s.</p>
     **/
    protected java.util.Set<Poly> polys =
        Collections.synchronizedSet(new HashSet<Poly>());

    /**
     * <p>The single {@link MapGUI.Path}.</p>
     **/
    protected Path path = new Path();

    /**
     * <p>The single {@link MapGUI.Graph}.</p>
     **/
    protected Graph graph = new Graph();

    /**
     * <p>The single {@link MapGUI.ParticleCloud}.</p>
     **/
    protected ParticleCloud cloud = new ParticleCloud();

    /**
     * <p>Consruct a new MapGUI.</p>
     *
     * <p>See <code>LocalNavigation.SonarGUI(int, double, double)</code>.</p>
     **/
    public MapGUI(int poseSaveInterval, double maxTV, double maxRV) {
        super(poseSaveInterval, maxTV, maxRV);
    }

    /**
     * <p>See <code>LocalNavigation.SonarGUI(int)</code>.</p>
     **/
    public MapGUI(int poseSaveInterval) {
        super(poseSaveInterval);
    }

    /**
     * <p>See <code>LocalNavigation.SonarGUI()</code>.</p>
     **/
    public MapGUI() {
        super();
    }

    /**
     * {@inheritDoc}
     *
     * <p>Default impl returns {@link #APPNAME}.</p>
     *
     * @return the title for the GUI frame
     **/
    @Override public String getAppName() {
        return APPNAME;
    }

    /**
     * <p>Add a rect for display.</p>
     * 
     * @param x the llc x coord in world frame (m)
     * @param y the llc y coord in world frame (m)
     * @param width the width in world frame (m)
     * @param height the height in world frame (m)
     * @param filled whether to fill the rect
     * @param color the rect color or null to use current
     **/
    public void addRect(double x, double y, double width, double height,
			boolean filled, Color color) {

        synchronized (rects) {

            if (color != null)
                rectColor = dupColor(color);

            rects.add(new Rect(x, y, width, height, filled, rectColor));
        }

        repaint();
    }

    /**
     * <p>Add a rect for display.</p>
     *
     * @param r the rect
     * @param filled whether to fill the rect
     * @param color the rect color or null to use current
     **/
    public void addRect(Rectangle2D.Double r, boolean filled, Color color) {
        synchronized (rects) {

            if (color != null)
                rectColor = dupColor(color);

            rects.add(new Rect(r, filled, rectColor));
        }

        repaint();
    }

    /**
     * <p>Add a polygon for display.</p>
     *
     * @param vertices the vertices in ccw order
     * @param closed whether to close the poly
     * @param filled whether to fill the poly
     * @param color the poly color or null to use current
     **/
    public void addPoly(java.util.List<Point2D.Double> vertices,
			boolean closed, boolean filled, Color color) {
        synchronized (polys) {

            if (color != null)
                polyColor = dupColor(color);

            if ( polys.add(new Poly(vertices, closed, filled, polyColor)) ) {

                System.err.println("added poly with " + vertices.size() + " verts");
                for (Point2D.Double vertex : vertices)
                    System.err.println("  " + vertex);
            }
        }

        repaint();
    }

    /**
     * <p>Erase all previously plotted rects.</p>
     **/
    public void eraseRects() {
        rects.clear();
        repaint();
    }

    /**
     * <p>Erase all previously polys.</p>
     **/
    public void erasePolys() {
        polys.clear();
        repaint();
    }

    /**
     * {@inheritDoc}
     *
     * <p>This impl {@link #paintRects}, {@link #paintPolys}, iff each is
     * enabled.</p>
     **/
    @Override protected void paintInWorldOverGridUnderPosesHook(Graphics2D g2d) {

        super.paintInWorldOverGridUnderPosesHook(g2d);

        if (rectsEnabled)
            paintRects(g2d);

        if (polysEnabled)
            paintPolys(g2d);

        paintPath(g2d);

        paintGraph(g2d);

        paintCloud(g2d);
    }

    /**
     * <p>Paint all {@link #rects}.</p>
     *
     * @param g2d the graphics context
     **/
    protected void paintRects(Graphics2D g2d) {

        //avoid NPE on init
        if (rects == null)
            return;

        setLineWidth(g2d, RECT_LINE_WIDTH);

        synchronized (rects) {
            for (Iterator<Rect> it = rects.iterator(); it.hasNext(); )
                it.next().paint(g2d);
        }
    }

    /**
     * <p>Paint all {@link #polys}.</p>
     *
     * @param g2d the graphics context
     **/
    protected void paintPolys(Graphics2D g2d) {

        //avoid NPE on init
        if (polys == null)
            return;

        setLineWidth(g2d, POLY_LINE_WIDTH);

        synchronized (polys) {

            for (Iterator<Poly> it = polys.iterator(); it.hasNext(); )
                it.next().paint(g2d);
        }
    }

    /**
     * <p>Paint current {@link #path}.</p>
     *
     * @param g2d the graphics context
     **/
    protected void paintPath(Graphics2D g2d) {
        setLineWidth(g2d, PATH_LINE_WIDTH);

        synchronized (path) {
            path.paint(g2d);
        }
    }

    /**
     * <p>Paint current {@link #graph}.</p>
     *
     * @param g2d the graphics context
     **/
    protected void paintGraph(Graphics2D g2d) {
        setLineWidth(g2d, GRAPH_LINE_WIDTH);

        synchronized (graph) {
            graph.paint(g2d);
        }
    }

    /**
     * <p>Paint current {@link #cloud}.</p>
     *
     * @param g2d the graphics context
     **/
    protected void paintCloud(Graphics2D g2d) {

        synchronized(cloud) {
            cloud.paint(g2d);
        }
    }

    private Subscriber<gui_msgs.GUIRectMsg> guiRectSub;
    private Subscriber<gui_msgs.GUIPolyMsg> guiPolySub;
    private Subscriber<gui_msgs.GUIEraseMsg> guiEraseSub;
    private Subscriber<gui_msgs.GUIPathMsg> guiPathSub;
    private Subscriber<gui_msgs.GUIGraphMsg> guiGraphSub;
    private Subscriber<gui_msgs.GUIParticleCloudMsg> guiLocSub;

    /**
     * Hook called by ROS to start the gui
     **/
    @Override public void onStart(ConnectedNode node) {
        guiRectSub = node.newSubscriber("gui/Rect", "gui_msgs/GUIRectMsg");
        guiRectSub.addMessageListener(new RectMessageListener(this));
        guiPolySub = node.newSubscriber("gui/Poly", "gui_msgs/GUIPolyMsg");
        guiPolySub.addMessageListener(new PolyMessageListener(this));
        guiEraseSub = node.newSubscriber("gui/Erase", "gui_msgs/GUIEraseMsg");
        guiEraseSub.addMessageListener(new EraseMessageListener(this));
        guiPathSub = node.newSubscriber("gui/Path", "gui_msgs/GUIPathMsg");
        guiPathSub.addMessageListener(
            new MessageListener<gui_msgs.GUIPathMsg>() {
                @Override public void onNewMessage(gui_msgs.GUIPathMsg message) {
                    // Deconstruct message into ArrayList<Point2D.Double>
                    ArrayList<Point2D.Double> pathList = new ArrayList<Point2D.Double>();
                    for (PointDataMsg point : message.getPoints()) {
                        pathList.add(new Point2D.Double(point.getX(), point.getY()));
                    }
                    path.updatePath(pathList);
                }
            });
        guiGraphSub = node.newSubscriber("gui/Graph", "gui_msgs/GUIGraphMsg");
        guiGraphSub.addMessageListener(
            new MessageListener<gui_msgs.GUIGraphMsg>() {
                @Override public void onNewMessage(gui_msgs.GUIGraphMsg message) {
                    // Deconstruct message into HashMap<Point2D.Double, ArrayList<Point2D.Double>>
                    HashMap<Point2D.Double, ArrayList<Point2D.Double>> map =
                        new HashMap<Point2D.Double, ArrayList<Point2D.Double>>();
                    for (PointMappingMsg mapping : message.getMappings()) {
                        ArrayList<Point2D.Double> targets = new ArrayList<Point2D.Double>();
                        for (PointDataMsg point : mapping.getTargets()) {
                            targets.add(new Point2D.Double(point.getX(), point.getY()));
                        }
                        Point2D.Double sourcePt = new Point2D.Double(
                            mapping.getSource().getX(), mapping.getSource().getY());
                        map.put(sourcePt, targets);
                    }
                    graph.updateGraph(map);
                }
            });
        guiLocSub = node.newSubscriber("gui/ParticleCloud", "gui_msgs/GUIParticleCloudMsg");
        guiLocSub.addMessageListener(
            new MessageListener<gui_msgs.GUIParticleCloudMsg>() {
                @Override public void onNewMessage(gui_msgs.GUIParticleCloudMsg message) {
                    List<PointDataMsg> points = message.getPoints();
                    double[] weights = message.getWeights();
                    List<Point2D.Double> convertedPoints = new ArrayList<Point2D.Double>();
                    if (points.size() != weights.length) {
                        throw new RuntimeException("points and weights length must match");
                    }
                    for (int i = 0; i < points.size(); i++) {
                        PointDataMsg ptData = points.get(i);
                        Point2D.Double pt = new Point2D.Double(ptData.getX(), ptData.getY());
                        convertedPoints.add(pt);
                    }
                    synchronized(cloud) {
                        cloud.updateParticleCloud(convertedPoints, weights);
                    }
                }
            });
        super.onStart(node);
    }


    /**
     * {@inheritDoc}
     *
     * <p>This impl tests the map graphics.</p>
     **/
    @Override public void testGraphicsHook() throws InterruptedException {

        super.testGraphicsHook();

        //TBD
    }
}
