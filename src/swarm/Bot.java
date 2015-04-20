package swarm;

import java.util.LinkedList;

/**
 *
 * @author chanjustin
 */
public class Bot {
    int size;
    double x;
    double y;
    
    public Bot(double x, double y, int size)
    {
        this.x = x;
        this.y = y;
        this.size = size;
    }
    
    
    //http://stackoverflow.com/questions/8367512/algorithm-to-detect-if-a-circles-intersect-with-any-other-circle-in-the-same-pla
    public boolean overlaps(LinkedList<Bot> bots)
    {
        for(Bot bot : bots)
        {
            if(Math.pow(bot.size-this.size,2) <= Math.pow(bot.x-this.x,2)+Math.pow(bot.y-this.y,2) &&
               Math.pow(bot.x-this.x,2)+Math.pow(bot.y-this.y,2) <= Math.pow(bot.size+this.size,2))
            {
                return true;
            }
        }
        return false;
    }
}
