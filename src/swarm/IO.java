package swarm;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Random;

/**
 *
 * @author chanjustin
 */
public class IO {
    static ArrayList<Bot> bots;
    static HashMap<Bot,Point2D> botCoords; //real coords relative to origin of image
    static HashMap<Bot,Point2D> graphicsCoords; //real coords relative to canvas
    static Random random = new Random(20);
    static int neighborDist = 15;
    static double epsilon = 0.00001;
    
    //simulates random noise from sensor data
    public static int getRandomSeed()
    {
        return random.nextInt(100);
    }
    
    //simulates sensors searching for nearby neighbors
    public static ArrayList<Bot> getNeighbors(Bot bot)
    {
        ArrayList<Bot> neighbors = new ArrayList<Bot>();
        Point2D coords = botCoords.get(bot);
        int neighborDistSquared = neighborDist*neighborDist;
        
        for(Bot b : bots)
        {
            double dist = botCoords.get(b).distanceSq(coords);
            if(dist < neighborDistSquared &&
               dist > epsilon)
            {
                neighbors.add(b);
            }
        }
        return neighbors;
    }
    
    public static double measuredDistance(Bot b1, Bot b2)
    {
        return botCoords.get(b1).distance(botCoords.get(b2));
    }
    
    public static Bot closestNeighbor(Bot bot)
    {
        Point2D coords = botCoords.get(bot);
        
        double dist = Double.MAX_VALUE;
        Bot minBot = bots.get(0);
        for(Bot b : bots)
        {
            Point2D point = botCoords.get(b);
            double d = point.distanceSq(coords);
            if(d < dist && d >= neighborDist)
            {
                dist = d;
                minBot = b;
            }
        }
        return minBot;
    }
    
    public static void move(Bot bot, double dx, double dy)
    {
        Point2D coords = botCoords.get(bot);
        coords.setLocation(coords.getX()+dx,coords.getY()+dy);
        
        Point2D gCoords = graphicsCoords.get(bot);
        gCoords.setLocation(gCoords.getX()+dx,gCoords.getY()-dy);
    }
}
