package swarm;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Random;

/**
 *
 * @author chanjustin
 */
public class Sensors {
    static ArrayList<Bot> bots;
    static ArrayList<Point2D> botCoords;
    static Random random = new Random(20);
    static final int neighborDist = 10;
    
    //simulates random noise from sensor data
    public static int getRandomSeed()
    {
        return random.nextInt();
    }
    
    //simulates sensors searching for nearby neighbors
    public static ArrayList<Bot> getNeighbors(Bot bot)
    {
        ArrayList<Bot> neighbors = new ArrayList<Bot>();
        Point2D coords = botCoords.get(bots.indexOf(bot));
        int neighborDistSquared = neighborDist*neighborDist;
        
        for(int i = 0; i < botCoords.size(); i++)
        {
            if(botCoords.get(i).distanceSq(coords) < neighborDistSquared)
            {
                neighbors.add(bots.get(i));
            }
        }
        return neighbors;
    }
    
    public static double measuredDistance(Bot b1, Bot b2)
    {
        return botCoords.get(bots.indexOf(b1)).distance( botCoords.get(bots.indexOf(b2)));
    }
    
    public static Bot closestNeighbor(Bot bot)
    {
        Point2D coords = botCoords.get(bots.indexOf(bot));
        
        double dist = Double.MAX_VALUE;
        int minInd = 0;
        int ind = 0;
        for(Point2D point : botCoords)
        {
            double d = point.distanceSq(coords);
            if(d < dist)
            {
                dist = d;
                minInd = ind;
            }
            ind += 1;
        }
        return bots.get(minInd);
    }
}
