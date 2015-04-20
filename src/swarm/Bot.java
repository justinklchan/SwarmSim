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
public class Bot implements Runnable {
    
    public enum State {
        START,WAIT_TO_MOVE,MOVE_WHILE_OUTSIDE,
        MOVE_WHILE_INSIDE,JOINED_SHAPE;
    }
    
    //robot's position is based on its localization
    //algorithm, position is not known a priori
    Point2D position;
    boolean seed = false;
    int[][] img;
    int seqNum;
    
    boolean gradientSeed = false;
    int gradientValue;
        
    //localization
    boolean stationary = true;
    boolean localized;
    
    boolean stopLocalization;
    boolean stopGradientFormation;
    boolean stopEdgeFollow;
    boolean stopIdGen;
    
    boolean localizationStarted;
    boolean gradientFormationStarted;
    boolean idGenStarted;
    boolean edgeFollowStarted;
   
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
        final int DESIRED_DISTANCE = 15;
        
        public void run()
        {
            System.out.println("edge follow "+seqNum);
            double prev = DISTANCE_MAX;
            while(!stopEdgeFollow)
            {
                delay();
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
                        //move straight forward (to the right)
                        IO.move(Bot.this,1,0);
//                        position.setLocation(position.getX()+1, position.getY());
//                        System.out.println("m1");
                    }
                    else
                    {
                        //move forward and counterclockwise, straight and up
                        IO.move(Bot.this,1,-1);
//                        position.setLocation(position.getX()+1, position.getY()-1);
//                        System.out.println("m2");
                    }
                }
                else
                {
                    if(prev > current)
                    {
                        //move straight forward
                        IO.move(Bot.this,1,0);
//                        position.setLocation(position.getX()+1, position.getY());
//                        System.out.println("m3");
                    }
                    else
                    {
                        //move forward and clockwise
                        IO.move(Bot.this,1,1);
//                        position.setLocation(position.getX()+1, position.getY()+1);
//                        System.out.println("m4");
                    }
                }
                prev = current;
                delay();
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
            System.out.println("gradient formation "+seqNum);
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
                }
                delay();
            }
        }
    }
    
    public class Localization implements Runnable
    {
        public void run()
        {
            if(!seed)
            {
                position = new Point2D.Double(0, 0);
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
//                        if(c == 0)
//                        {
//                            c = 1;
//                        }
                        //this is a vector
                        Point2D v = new Point2D.Double(
                                       (position.getX()-bot.position.getX())/c,
                                       (position.getY()-bot.position.getY())/c);
                        double measuredDist = IO.measuredDistance(Bot.this,bot);
                        Point2D n = new Point2D.Double(bot.position.getX()+measuredDist*v.getX(),
                                                       bot.position.getY()+measuredDist*v.getY());
                        position = new Point2D.Double(position.getX()-(position.getX()-n.getX())/4,
                                                      position.getY()-(position.getY()-n.getY())/4);
//                        System.out.println("localized "+position.getX()+","+position.getY());
                    }
                }
                delay();
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
                Bot[] bots = set.toArray(new Bot[set.size()]);
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
    
    public class GenerateLocallyUniqueID implements Runnable
    {
        boolean idGenerated = false;
        
        public void run()
        {
            while(!stopIdGen)
            {
                if(!idGenerated)
                {
                    int randSeed = IO.getRandomSeed();
                    Random random = new Random(randSeed);
                    id = random.nextInt(10000);
                    idGenerated = true;
                    System.out.println("id: "+id);
                }
                else
                {
                    //detects if id is no longer locally unique
                    ArrayList<Bot> neighbors = IO.getNeighbors(Bot.this);
                    for(Bot bot : neighbors)
                    {
                        if(id == bot.id)
                        {
                            idGenerated = false;
                        }
                    }
                }
                delay();
            }
        }
    }
    
    public void delay()
    {
        try
        {
            Thread.sleep(100);
        }
        catch(Exception e)
        {
            e.printStackTrace();
        }
    }
    
    public void run()
    {
        stopLocalization = false;
        stopGradientFormation = false;
        stopEdgeFollow = false;
        stopIdGen = false;

        localizationStarted = false;
        gradientFormationStarted = false;
        idGenStarted = false;
        edgeFollowStarted = false;
        
        int startupTime = 1;
        int yieldDistance = 15;

        Thread edgeFollow = null;
        Thread gradientFormation = null;
        Thread localization = null;
        Thread idGen = null;
    
        stationary = true;
        State state = State.START;
        int timer = 0;
        
        try
        {
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
                            gradientFormation.start();
                            gradientFormationStarted = true;
                        }
                        if(!localizationStarted)
                        {
                            stopLocalization = false;
                            localization = new Thread(new Localization());
                            localization.start();
                            localizationStarted = true;
                        }
                        if(!idGenStarted)
                        {
                            idGen = new Thread(new GenerateLocallyUniqueID());
                            idGen.start();
                            idGenStarted = true;
                        }
                        timer += 1;
                        Thread.sleep(1000);
                        System.out.println(timer);
                        if(timer > startupTime)
                        {
                            state = State.WAIT_TO_MOVE;
                        }
                    }
                }
                else if(state == State.WAIT_TO_MOVE)
                {
                    ArrayList<Bot> neighbors = IO.getNeighbors(Bot.this);
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
//                    System.out.println("outside");
                    if(img[(int)position.getY()][(int)position.getX()] == 0)
                    {
                        state = State.MOVE_WHILE_INSIDE;
                    }
                    
                    //TODO
    //                if( > yieldDistance)
    //                {
                        stopEdgeFollow = false;
                        stationary = false;
                        if(!edgeFollowStarted)
                        {
                            edgeFollow = new Thread(new EdgeFollow());
                            edgeFollow.start();
                            edgeFollowStarted = true;
                        }
    //                }
    //                else
    //                {
    //                    stopEdgeFollow = true;
    //                    if(edgeFollow != null)
    //                    {
    //                        edgeFollow.join();
    //                    }
    //                    stationary = true;
    //                }
                }
                else if(state == State.MOVE_WHILE_INSIDE)
                {
//                    System.out.println("inside");
                    System.out.println(img[(int)position.getY()][(int)position.getX()]);
                    if(img[(int)position.getY()][(int)position.getX()] == 1)
                    {   
//                        System.out.println("JOINED SHAPE!!!!");
                        state = State.JOINED_SHAPE;
                    }
                    if(gradientValue <= IO.closestNeighbor(Bot.this).gradientValue)
                    {
                        state = State.JOINED_SHAPE;
                    }
                    //TODO
    //                if( > yieldDistance)
    //                {
                        stopEdgeFollow = false;
                        stationary = false;
                        if(!edgeFollowStarted)
                        {
                            edgeFollow = new Thread(new EdgeFollow());
                            edgeFollow.start();
                            edgeFollowStarted = true;
                        }
    //                }
    //                else
    //                {
    //                    stopEdgeFollow = true;
    //                    if(edgeFollow != null)
    //                    {
    //                        edgeFollow.join();
    //                    }
    //                    stationary = true;
    //                }
                }   
                else if(state == State.JOINED_SHAPE)
                {
                    stationary = true;
                    if(edgeFollow != null)
                    {
                        edgeFollow.join();
                    }
                    stopLocalization = true;
                    if(localization != null)
                    {
                        localization.join();
                    }
                    stopGradientFormation = true;
                    if(gradientFormation != null)
                    {
                        gradientFormation.join();
                    }
                    stopIdGen = true;
                    if(idGen != null)
                    {
                        idGen.join();
                    }
                    break;
                }
            }
        }
        catch(Exception e)
        {
            e.printStackTrace();
        }
        System.out.println(seqNum + " ended");
    }
}
