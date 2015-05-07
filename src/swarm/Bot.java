package swarm;

import java.awt.geom.Point2D;
import java.text.DecimalFormat;
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
    
    //robot's position is based on its localization
    //algorithm, position is not known a priori
    Random random;
    Point2D position;
    boolean seed;
    int[][] img;
    int seqNum;
    int s;
    DecimalFormat df = new DecimalFormat("0.##");
    Point2D direction;
    boolean ended = false;
    
    boolean gradientSeed = false;
    int gradientValue;
        
    //localization
    boolean stationary = true;
    boolean localized;
    
    boolean stopLocalization = true;
    boolean stopGradientFormation = true;
    boolean stopEdgeFollow = true;
    boolean stopIdGen = true;
   
    int id;
    double angle = 20;
    
    public Bot(int[][] img, int s, boolean seed) 
    {
        this.img = new int[img.length][];
        for(int i = 0; i < img.length; i++)
        {
            this.img[i] = img[i].clone();
        }
        this.s = s;
        //robots are all facing to the right
        direction = new Point2D.Double(1,0);
        this.seed = seed;
        if(!seed)
        {
            position = new Point2D.Double(IO.epsilon,IO.epsilon);
        }
    }
    
    final int DISTANCE_MAX = Integer.MAX_VALUE;
    final int DESIRED_DISTANCE = 6;
    final double moveIncrement = 1;

    double prev = DISTANCE_MAX;
    public void EdgeFollow()
    {
        double current = DISTANCE_MAX;
        ArrayList<Bot> neighbors = IO.getNeighbors(Bot.this);
        for(Bot neighbor : neighbors)
        {
            double dist = IO.measuredDistance(Bot.this,neighbor);
            if(dist < current && neighbor.stationary)
            {
                current = dist;
            }
        }
        
        if(current < DESIRED_DISTANCE)
        {
            if(prev < current)
            {
                //move straight forward (to the right) 
                IO.move(Bot.this,direction.getX(),direction.getY());
//                System.out.println("m1");
            }
            else
            {
                //move forward and counterclockwise, straight and up
                double xp = direction.getX()*Math.cos(-angle)-direction.getY()*Math.sin(-angle);
                double yp = direction.getX()*Math.sin(-angle)+direction.getY()*Math.cos(-angle);
                direction = new Point2D.Double(xp, yp);
                IO.move(Bot.this,direction.getX(),direction.getY());
//                System.out.println("m2");
            }
        }
        else
        {
            if(prev > current)
            {
                //move straight forward
                IO.move(Bot.this,direction.getX(),direction.getY());
//                System.out.println("m3");
            }
            else
            {
                //move forward and clockwise
                double xp = direction.getX()*Math.cos(angle)-direction.getY()*Math.sin(angle);
                double yp = direction.getX()*Math.sin(angle)+direction.getY()*Math.cos(angle);
                direction = new Point2D.Double(xp, yp);
                direction = new Point2D.Double(direction.getX(), direction.getY());
                IO.move(Bot.this,direction.getX(),direction.getY());
//                System.out.println("m4");
            }
        }
        prev = current;
    }
    
    //gradient formation
    final int GRADIENT_MAX = 1000; //max gradient == maxbots
    final int G = 10;

    public boolean GradientFormation()
    {
        boolean modified = false;
        int oldGradient = gradientValue;
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
            if(gradientValue != oldGradient)
            {
                modified = true;
            }
        }
        return modified;
    }
    
    public boolean Localization()
    {
//                System.out.println("LOCALIZE");
        boolean modified = false;
        Point2D oldPos = (Point2D)position.clone();
        position = IO.botCoords.get(Bot.this);
        
        if(Math.abs(oldPos.getX()-position.getX()) > IO.epsilon || 
           Math.abs(oldPos.getY()-position.getY()) > IO.epsilon)
        {
            modified = true;
        }
//        ArrayList<Bot> neighbors = IO.getNeighbors(Bot.this);
//        ArrayList<Bot> nList = new ArrayList<Bot>();
//        for(Bot bot : neighbors)
//        {
//            if(bot.localized && bot.stationary)
//            {
//                nList.add(bot);
//            }
//        }
//        if(nList.size() >= 3)
////                if(has3NonCollinearBots(nList))
//        {
//            for(Bot bot : nList)
//            {
//                double c = position.distance(bot.position);
//                //this is a vector
//                Point2D v = new Point2D.Double(
//                               (position.getX()-bot.position.getX())/c,
//                               (position.getY()-bot.position.getY())/c);
//                double measuredDist = IO.measuredDistance(Bot.this,bot);
//                Point2D n = new Point2D.Double(bot.position.getX()+measuredDist*v.getX(),
//                                               bot.position.getY()+measuredDist*v.getY());
//                position = new Point2D.Double(position.getX()-(position.getX()-n.getX())/4,
//                                              position.getY()-(position.getY()-n.getY())/4);
//
////                        Point2D realCoords = IO.botCoords.get(Bot.this);
////                        System.out.println("localized ("+df.format(realCoords.getX())+","+df.format(realCoords.getY())+"), "+
////                                "("+df.format(position.getX())+","+df.format(position.getY())+")");
//            }
//        }
        return modified;
    }
        
    //http://stackoverflow.com/questions/12548312/find-all-subsets-of-length-k-in-an-array
    private void getSubsets(List<Integer> superSet, int k, int idx, Set<Integer> current,List<Set<Integer>> solution) {
        //successful stop clause
        if (current.size() == k) {
            solution.add(new HashSet<Integer>(current));
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
        List<Set<Integer>> res = new ArrayList<Set<Integer>>();
        getSubsets(superSet, k, 0, new HashSet<Integer>(), res);
        return res;
    }

    public boolean has3NonCollinearBots(ArrayList<Bot> nList)
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
            if(isNonCollinear(bots[0].position,bots[1].position,bots[2].position))
            {
                return true;
            }
        }
        return false;
    }
    
    //http://math.stackexchange.com/questions/405966/if-i-have-three-points-is-there-an-easy-way-to-tell-if-they-are-collinear
    public boolean isNonCollinear(Point2D p1, Point2D p2, Point2D p3)
    {
        return (p2.getY()-p1.getY())*(p1.getX()-p2.getX()) != (p3.getY()-p2.getY())*(p2.getX()-p1.getX());
    }

    boolean idGenerated = false;

    public void GenerateLocallyUniqueID()
    {
        if(!idGenerated)
        {
            int randSeed = IO.getRandomSeed();
            random = new Random(randSeed);
            id = random.nextInt(10000);
            idGenerated = true;
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
    }
    
    public void delay()
    {
        try
        {
            Thread.sleep(10);
        }
        catch(Exception e)
        {
            e.printStackTrace();
        }
    }

    int yieldDistance = 10;
    State state = State.START;
        
    public void run()
    {
        try
        {
            System.out.println(seqNum+" "+gradientValue);
            if(state == State.START)
            {
                if(seed)
                {
                    state = State.JOINED_SHAPE;
                }
                else
                {
                    stopGradientFormation = false;
                    stopLocalization = false;
                    stopIdGen = false;
                    state = State.WAIT_TO_MOVE;
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
                        if(bot.gradientValue > h)
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
                            if(gradientValue == bot.gradientValue && 
                               id <= bot.id)
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
                int val = valAt((int)position.getY(),(int)position.getX());
//                    System.out.println("outside "+id);
//                    System.out.println("("+(int)position.getX()+","+(int)position.getY()+") " + val);
                if(val == 0)
                {
                    state = State.MOVE_WHILE_INSIDE;
                }

                if(distToNearestEdgeFollowingBot() > yieldDistance)
                {
                    stopEdgeFollow = false;
                    stationary = false;
                }
                else
                {
                    stopEdgeFollow = true;
                    stationary = true;
                }
            }
            else if(state == State.MOVE_WHILE_INSIDE)
            {
                int val = valAt((int)position.getY(),(int)position.getX());
//                    System.out.println("inside");
//                    System.out.println("("+(int)position.getX()+","+(int)position.getY()+") " + val);

                if(val == 1)
                {   
                    System.out.println("JOINED SHAPE 1");
                    state = State.JOINED_SHAPE;
                }
                if(gradientValue <= IO.closestNeighbor(Bot.this).gradientValue)
                {
                    System.out.println("JOINED SHAPE 2");
                    state = State.JOINED_SHAPE;
                }

                if(distToNearestEdgeFollowingBot() > yieldDistance)
                {
                    stopEdgeFollow = false;
                    stationary = false;
                }
                else
                {
                    stopEdgeFollow = true;
                    stationary = true;
                }
            }   
            else if(state == State.JOINED_SHAPE)
            {
                stopEdgeFollow = true;
                System.out.println("EDGE FOLLOW STOPPED");

                stopLocalization = true;
                System.out.println("LOCALIZATION STOPPED");

                stopGradientFormation = true;
                System.out.println("GRADIENT FORMATION STOPPED");

                stopIdGen = true;
                System.out.println("ID GEN STOPPED");
                
                if(!seed)
                {
                    gradientValue = 0;
                }
                stationary = true;
                ended = true;
                System.out.println(seqNum + " ended");
            }
        }
        catch(Exception e)
        {
            e.printStackTrace();
        }
    }
    
    
    public double distToNearestEdgeFollowingBot()
    {
//        ArrayList<Bot> neighbors = IO.getNeighbors(Bot.this);
//        double closest = Double.MAX_VALUE;
//        for(Bot bot : neighbors)
//        {
//            if(!bot.stationary)
//            {
//                double dist = IO.measuredDistance(Bot.this,bot);
//                if(dist < closest)
//                {
//                    closest = dist;
//                }
//            }
//        }
//        return closest;
        return 20;
    }
    
    public int valAt(int y, int x)
    {
        if(x < 0 || y < 0 || y >= img.length || x >= img[0].length)
        {
            return 1;
        }
        else
        {
            return img[img.length-y-1][x];
        }
    }
}
