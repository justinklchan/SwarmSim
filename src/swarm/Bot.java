package swarm;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Random;
import java.util.Set;

/**
 *
 * @author chanjustin
 */
public class Bot {
    
    public enum State {
        START,WAIT_TO_MOVE,MOVE_WHILE_OUTSIDE,
        MOVE_WHILE_INSIDE,JOINED_SHAPE;
    }
    
    //robot's position is based on its localization
    //algorithm, position is not known a priori
    Point2D position;
    boolean seed = false;
    int[][] img;
    
    boolean gradientSeed = false;
    int gradientValue;
        
    //localization
    boolean stationary = true;
    boolean localized;
    
    boolean stopLocalization = false;
    boolean stopGradientFormation = false;
    boolean stopEdgeFollow = false;
    boolean localizationStarted = true;
    boolean gradientFormationStarted = true;
    int startupTime = 60;
    int yieldDistance = 15;
    
    int id;
    
    public Bot(int[][] img)
    {
        this.img = new int[img.length][];
        for(int i = 0; i < img.length; i++)
        {
            this.img[i] = img[i].clone();
        }
    }
    
    public class EdgeFollow implements Runnable
    {
        final int DISTANCE_MAX = 10;
        final int DESIRED_DISTANCE = 5;
        final int MOVEMENT_ANGLE = 45;
        
        public void run()
        {
            double prev = DISTANCE_MAX;
            while(!stopEdgeFollow)
            {
                double current = DISTANCE_MAX;
                ArrayList<Bot> neighbors = IO.getNeighbors(Bot.this);
                for(Bot neighbor : neighbors)
                {
                    double dist = IO.measuredDistance(Bot.this,neighbor);
                    if(dist < current)
                    {
                        current = dist;
                    }
                }
                if(current < DESIRED_DISTANCE)
                {
                    if(prev < current)
                    {
                        //move straight forward
                        IO.move(Bot.this,0);
                    }
                    else
                    {
                        //move forward and counterclockwise
                        IO.move(Bot.this,MOVEMENT_ANGLE);
                    }
                }
                else
                {
                    if(prev > current)
                    {
                        //move straight forward
                        IO.move(Bot.this,0);
                    }
                    else
                    {
                        //move forward and clockwise
                        IO.move(Bot.this,-MOVEMENT_ANGLE);
                    }
                }
                prev = current;
            }
        }
    }
    
    public class GradientFormation implements Runnable
    {
        //gradient formation
        final int GRADIENT_MAX = 1000; //max gradient == maxbots
        final int G = 20;
    
        public void run()
        {
            while(!stopGradientFormation)
            {
                gradientFormationStarted = true;
                if(gradientSeed)
                {
                    gradientValue = 0;
                }
                else
                {
                    gradientValue = GRADIENT_MAX;
                    ArrayList<Bot> neighbors = IO.getNeighbors(Bot.this);
                    for(Bot neighbor : neighbors)
                    {
                        if(IO.measuredDistance(Bot.this,neighbor) < G)
                        {
                            if(neighbor.gradientValue < gradientValue)
                            {
                                gradientValue = neighbor.gradientValue;
                            }
                        }
                    }
                    gradientValue += 1;
                    //transmit our gradient value
                }
            }
        }
    }
    
    public class Localization implements Runnable
    {
        public void run()
        {
            if(!seed)
            {
                position.setLocation(0, 0);
            }
            while(!stopLocalization)
            {
                localizationStarted = true;
                ArrayList<Bot> neighbors = IO.getNeighbors(Bot.this);
                ArrayList<Bot> nList = new ArrayList<Bot>();
                for(Bot bot : neighbors)
                {
                    if(bot.localized && bot.stationary)
                    {
                        nList.add(bot);
                    }
                }
                if(has3CollinearBots(nList))
                {
                    for(Bot bot : nList)
                    {
                        double c = position.distance(bot.position);

                        //this is a vector
                        Point2D v = new Point2D.Double(
                                       (position.getX()-bot.position.getX())/c,
                                       (position.getY()-bot.position.getY())/c);
                        double measuredDist = IO.measuredDistance(Bot.this,bot);
                        Point2D n = new Point2D.Double(bot.position.getX()+measuredDist*v.getX(),
                                                       bot.position.getY()+measuredDist*v.getY());
                        position = new Point2D.Double(position.getX()-(position.getX()-bot.position.getX())/4,
                                                      position.getY()-(position.getY()-bot.position.getY())/4);
                    }
                }
            }
        }
        
        //http://stackoverflow.com/questions/12548312/find-all-subsets-of-length-k-in-an-array
        private void getSubsets(List<Integer> superSet, int k, int idx, Set<Integer> current,List<Set<Integer>> solution) {
            //successful stop clause
            if (current.size() == k) {
                solution.add(new HashSet<>(current));
                return;
            }
            //unseccessful stop clause
            if (idx == superSet.size()) return;
            Integer x = superSet.get(idx);
            current.add(x);
            //"guess" x is in the subset
            getSubsets(superSet, k, idx+1, current, solution);
            current.remove(x);
            //"guess" x is not in the subset
            getSubsets(superSet, k, idx+1, current, solution);
        }

        //http://stackoverflow.com/questions/12548312/find-all-subsets-of-length-k-in-an-array
        public List<Set<Integer>> getSubsets(List<Integer> superSet, int k) {
            List<Set<Integer>> res = new ArrayList<>();
            getSubsets(superSet, k, 0, new HashSet<Integer>(), res);
            return res;
        }

        public boolean has3CollinearBots(ArrayList<Bot> nList)
        {
            //get all subsets of size 3 for [0...nList.size()]
            List<Integer>superSet = new ArrayList<Integer>();
            for(int i = 0; i < nList.size(); i++)
            {
                superSet.add(i);
            }
            List<Set<Integer>> subsets = getSubsets(superSet,3);

            //convert subset of indices to subset of Bot objects
            List<Set<Bot>> botSubsets = new ArrayList<Set<Bot>>();
            for(Set<Integer> set : subsets)
            {
                HashSet<Bot> botSet = new HashSet<Bot>();
                for(Integer i : set)
                {
                    botSet.add(nList.get(i));
                }
                botSubsets.add(botSet);
            }

            for(Set<Bot> set : botSubsets)
            {   
                Bot[] bots = (Bot[])set.toArray();
                if(isCollinear(bots[0].position,bots[1].position,bots[2].position))
                {
                    return true;
                }
            }
            return false;
        }
    }
    
    //http://math.stackexchange.com/questions/405966/if-i-have-three-points-is-there-an-easy-way-to-tell-if-they-are-collinear
    public boolean isCollinear(Point2D p1, Point2D p2, Point2D p3)
    {
        return (p2.getY()-p1.getY())*(p1.getX()-p2.getX()) == (p3.getY()-p2.getY())*(p2.getX()-p1.getX());
    }
    
    public void generateLocallyUniqueID()
    {
        boolean idGenerated = false;
        while(true)
        {
            if(!idGenerated)
            {
                int randSeed = IO.getRandomSeed();
                Random random = new Random(randSeed);
                id = random.nextInt();
            }
            else
            {
                ArrayList<Bot> neighbors = IO.getNeighbors(this);
                for(Bot bot : neighbors)
                {
                    if(id == bot.id)
                    {
                        idGenerated = false;
                    }
                }
            }
        }
    }
    
    public void selfAssembly() throws Exception
    {
        stationary = true;
        State state = State.START;
        int timer = 0;
        
        Thread edgeFollow = null;
        Thread gradientFormation = null;
        Thread localization = null;
        
        while(true)
        {
            if(state == State.START)
            {
                if(seed)
                {
                    state = State.JOINED_SHAPE;
                }
                else
                {
                    if(!gradientFormationStarted)
                    {
                        stopGradientFormation = false;
                        gradientFormation = new Thread(new GradientFormation());
                    }
                    if(!localizationStarted)
                    {
                        stopLocalization = false;
                        localization = new Thread(new Localization());
                    }
                    timer += 1;
                    if(timer > startupTime)
                    {
                        state = State.WAIT_TO_MOVE;
                    }
                }
            }
            else if(state == State.WAIT_TO_MOVE)
            {
                ArrayList<Bot> neighbors = IO.getNeighbors(this);
                boolean movingNeighbors = false;
                for(Bot bot : neighbors)
                {
                    if(!bot.stationary)
                    {
                        movingNeighbors = true;
                        break;
                    }
                }
                if(!movingNeighbors)
                {
                    int h = 0;
                    for(Bot bot : neighbors)
                    {
                        if(h < bot.gradientValue)
                        {
                            h = bot.gradientValue;
                        }
                    }
                    if(gradientValue > h)
                    {
                        state = State.MOVE_WHILE_OUTSIDE;
                    }
                    else if(gradientValue == h)
                    {
                        boolean condition = true;
                        for(Bot bot : neighbors)
                        {
                            if(gradientValue == bot.gradientValue && id <= bot.id)
                            {
                                condition = false;
                                break;
                            }
                        }
                        if(condition)
                        {
                            state = State.MOVE_WHILE_OUTSIDE;
                        }
                    }
                }
            }
            else if(state == State.MOVE_WHILE_OUTSIDE)
            {
                if(img[(int)position.getX()][(int)position.getY()] == 0)
                {
                    state = State.MOVE_WHILE_INSIDE;
                }
                if( > yieldDistance)
                {
                    stopEdgeFollow = false;
                    stationary = false;
                    edgeFollow = new Thread(new EdgeFollow());
                    edgeFollow.start();
                }
                else
                {
                    stopEdgeFollow = true;
                    edgeFollow.join();
                    stationary = true;
                }
            }
            else if(state == State.MOVE_WHILE_INSIDE)
            {
                if(img[(int)position.getX()][(int)position.getY()] == 1)
                {
                    state = State.JOINED_SHAPE;
                }
                if(gradientValue <= IO.closestNeighbor(this).gradientValue)
                {
                    state = State.JOINED_SHAPE;
                }
                if( > yieldDistance)
                {
                    stopEdgeFollow = false;
                    stationary = false;
                    edgeFollow = new Thread(new EdgeFollow());
                    edgeFollow.start();
                }
                else
                {
                    stopEdgeFollow = true;
                    edgeFollow.join();
                    stationary = true;
                }
            }   
            else if(state == State.JOINED_SHAPE)
            {
                stopLocalization = true;
                localization.join();
                stopGradientFormation = true;
                gradientFormation.join();
            }
        }
            
    }
}
