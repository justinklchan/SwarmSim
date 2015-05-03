package swarm;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.geom.AffineTransform;
import java.awt.geom.Point2D;
import java.awt.image.AffineTransformOp;
import java.awt.image.BufferedImage;
import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Random;
import javax.imageio.ImageIO;
import javax.swing.JPanel;
import javax.swing.Timer;

/**
 *
 * @author chanjustin
 */
public class Swarm extends javax.swing.JFrame {

    /**
     * Creates new form Swarm
     */
    public Swarm() {
        initComponents();
    }

    /**
     * This method is called from within the constructor to initialize the form.
     * WARNING: Do NOT modify this code. The content of this method is always
     * regenerated by the Form Editor.
     */
    @SuppressWarnings("unchecked")
    // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
    private void initComponents() {

        CustomPanel = new SimPanel();

        setDefaultCloseOperation(javax.swing.WindowConstants.EXIT_ON_CLOSE);

        javax.swing.GroupLayout CustomPanelLayout = new javax.swing.GroupLayout(CustomPanel);
        CustomPanel.setLayout(CustomPanelLayout);
        CustomPanelLayout.setHorizontalGroup(
            CustomPanelLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 400, Short.MAX_VALUE)
        );
        CustomPanelLayout.setVerticalGroup(
            CustomPanelLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 300, Short.MAX_VALUE)
        );

        javax.swing.GroupLayout layout = new javax.swing.GroupLayout(getContentPane());
        getContentPane().setLayout(layout);
        layout.setHorizontalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addComponent(CustomPanel, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
        );
        layout.setVerticalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addComponent(CustomPanel, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
        );

        pack();
    }// </editor-fold>//GEN-END:initComponents
    
    class SimPanel extends JPanel
    {
        int RAND_SEED = 20;
        Random random;
        boolean spawned = false;
        
        int nBots = 1000;
        int botSize = 5;
        int[][] img;
        
        Timer timer;
        ArrayList<Thread> botThreads;
        
        public SimPanel()
        {
            super();
            random = new Random(RAND_SEED);
            IO.bots = new ArrayList<Bot>(nBots);
            IO.botCoords = new HashMap<Bot,Point2D>(nBots);
            IO.graphicsCoords = new HashMap<Bot,Point2D>(nBots);
            botThreads = new ArrayList<Thread>(nBots);
            timer = new Timer(10, taskPerformer);
            timer.start();
        }
        
        ActionListener taskPerformer = new ActionListener() {
            public void actionPerformed(ActionEvent evt) {
                repaint();
            }
        };
        
        @Override
        public void paintComponent(Graphics g) {
            super.paintComponent(g);
            if(!spawned)
            {
                //numBots must be a square number
                int numBots = 4;
                int sBots = (int)Math.sqrt(numBots);
                int x = 105-(sBots-1)*5;
                int y = 95-(sBots-1)*5;
                int w = 5*sBots;
                int h = w;
                
                int s = 1;
                int sx = 110;
                int sy = 90;
                
                img = readShape(s,s);
                setSeed(sx,sy,s);
                packSpawnInArea(sx, sy+botSize, x, y, w, h, img, s);
                int i = 0;
                for(Bot bot : IO.bots)
                {
                    bot.seqNum = i;
                    i += 1;
                }
                spawned = true;
            }
            
            for(Bot bot : IO.bots)
            {
                if(!bot.ended)
                {
                    bot.run();
                }
            }
            
            drawShape(g,img,110,0);
            drawBots(g);
        }
        
        //TODO: this method should be synchronized
        public void drawBots(Graphics g)
        {
            for(Bot bot : IO.bots)
            {
                if(bot.seed)
                {
                    g.setColor(Color.green);
                }
                else
                {
                   // bots get redder, the nearer they are to the gradient seed
//                    int maxGradientValue = 0;
//                    for(Bot b : IO.bots)
//                    {
//                        if(b.gradientValue > maxGradientValue)
//                        {
//                            maxGradientValue = b.gradientValue;
//                        }
//                    }
//                    
//                    int cVal = 0;
//                    if(maxGradientValue != 0)
//                    {
//                        cVal = 255-((bot.gradientValue/maxGradientValue)*255);
//                    }
                    g.setColor(new Color(255,0,0));
                }
//                if(bot.gradientValue == 1)
//                {
//                    g.setColor(Color.BLUE);
//                }
//                if(bot.gradientValue == 2)
//                {
//                    g.setColor(Color.RED);
//                }
//                if(bot.gradientValue == 3)
//                {
//                    g.setColor(Color.PINK);
//                }
                g.drawOval((int)IO.graphicsCoords.get(bot).getX(), (int)IO.graphicsCoords.get(bot).getY(), botSize, botSize);
//                g.drawString(bot.gradientValue+"", (int)IO.graphicsCoords.get(bot).getX(), (int)IO.graphicsCoords.get(bot).getY());
            }
        }
        
        //we set 4 seeds, one is the gradient seed
        public void setSeed(int x, int y, int s)
        {
            int xIncrement = 5;
            int yIncrement = 5;
            int seedCount = 0;
            
            for(int i = 0; i <= 1; i += 1)
            {
                for(int j = 0; j <= 1; j += 1)
                {
                    Bot bot = new Bot(img, s, true);
                    bot.localized = true;
                    
//                    //this is the global coordinates
                    double gx = x+i*xIncrement;
                    double gy = y+j*yIncrement;
                    IO.graphicsCoords.put(bot, new Point2D.Double(gx,gy));
                        
                    double px = 0;
                    double py = 0;
                    if(seedCount == 0)
                    {
                        px = 0;
                        py = 5;
                    }
                    else if(seedCount == 1)
                    {
                        px = 0;
                        py = 0;
                    }
                    else if(seedCount == 2)
                    {
                        px = 5;
                        py = 5;
                    }
                    else
                    {
                        px = 5;
                        py = 0;
                    }
                    bot.position = new Point2D.Double(px,py);
                    IO.botCoords.put(bot,new Point2D.Double(px,py));
                    seedCount += 1;
                    
                    IO.bots.add(bot);
                }
            }
            IO.bots.get(IO.bots.size()-1).gradientSeed = true;
        }
        
        //pack bots in a given area
        public void packSpawnInArea(int sx, int sy, int startX, int startY, int width, int height, int[][] imgRep, int s)
        {
            int xTimes = width/botSize;
            int yTimes = height/botSize;
            
            for(int i = 0; i < xTimes; i++)
            {
                for(int j = 0; j < yTimes; j++)
                {
                    Bot bot = new Bot(imgRep,s,false);
                    IO.bots.add(bot);
                    double gx = startX+botSize*i;
                    double gy = (startY+botSize*j);
                    IO.graphicsCoords.put(bot, new Point2D.Double(gx,gy));
                    
                    double x = gx-sx;
                    double y = gy-sy;
                    IO.botCoords.put(bot,new Point2D.Double(x,y));
                }
            }
        }
        
        //draw shape
        public void drawShape(Graphics g, int[][] shape, int tx, int ty)
        {
            g.setColor(Color.black);
            for(int i = 0; i < shape.length; i++)
            {
                for(int j = 0; j < shape[i].length; j++)
                {
                    if(shape[i][j] == 0)
                    {
                        g.drawOval(j+tx, i+ty, 1, 1);
                    }
                }
            }
        }
        
        public void translate(int[][] shape, int tx, int ty)
        {
            if(tx > 0)
            {
                //for every row
                for(int i = 0; i < shape.length; i++)
                {
                    for(int j = shape[i].length-1-tx; j >= 0; j--)
                    {
                        //shift elements in y direction
                        shape[i][j+tx] = shape[i][j];
                        shape[i][j] = 1;
                    }
                }
            }
            
            if(ty > 0)
            {
                //for every row
                for(int i = shape.length-1; i >= ty; i--)
                {
                    for(int j = 0; j < shape[i].length; j++)
                    {
                        shape[i][j] = shape[i-ty][j];
                        shape[i-ty][j] = 1;
                    }
                }
            }
        }
        
        //reads shape from file as bufferedimage, stored as int[][]
        //image can be scaled by sx/sy
        //image can be translated by tx/ty
        //array is 0 for black, 1 for white
        public int[][] readShape(double sx, double sy)
        {
            BufferedImage image = null;
            try
            {
                image = ImageIO.read(new File("src/shape.bmp"));
                image = scale(image,sx,sy);
            }
            catch(Exception e)
            {
                e.printStackTrace();
            }
            int[][] shape = new int[image.getHeight()][image.getWidth()];
            for(int i = 0; i < image.getHeight(); i++)
            {
                for(int j = 0; j < image.getWidth(); j++)
                {
                    shape[i][j] = 1;
                }
            }
            
            //converting from bufferedimage to int[][]
            for(int i = 0; i < image.getHeight(); i++)
            {
                for(int j = 0; j < image.getWidth(); j++)
                {
                    Color c = convertToColor(image.getRGB(i, j));
                    if(c.equals(Color.black))
                    {
                        shape[j][i] = 0;
                    }
                }
            }
            return shape;
        }
        
        //scales image by a certain amount
        //http://stackoverflow.com/questions/4216123/how-to-scale-a-bufferedimage
        public BufferedImage scale(BufferedImage before, double sx, double sy)
        {
            int w = before.getWidth();
            int h = before.getHeight();
            BufferedImage after = new BufferedImage((int)(w*sx), (int)(h*sy), BufferedImage.TYPE_INT_ARGB);
            for(int i = 0; i < w; i++)
            {
                for(int j = 0; j < h; j++)
                {
                    after.setRGB(i, j, convertToRGB(Color.white));
                }
            }
            AffineTransform at = new AffineTransform();
            at.scale(sx, sy);
            AffineTransformOp scaleOp = 
               new AffineTransformOp(at, AffineTransformOp.TYPE_BILINEAR);
            after = scaleOp.filter(before, after);
            return after;
        }
        
        public int convertToRGB(Color c)
        {
            return ((c.getRed()&0x0ff)<<16)|((c.getGreen()&0x0ff)<<8)|(c.getBlue()&0x0ff);
        }
        
        public Color convertToColor(int c)
        {
            return new Color(c);
        }
    }
    
    /**
     * @param args the command line arguments
     */
    public static void main(String args[]) {
        /* Set the Nimbus look and feel */
        //<editor-fold defaultstate="collapsed" desc=" Look and feel setting code (optional) ">
        /* If Nimbus (introduced in Java SE 6) is not available, stay with the default look and feel.
         * For details see http://download.oracle.com/javase/tutorial/uiswing/lookandfeel/plaf.html 
         */
        try {
            for (javax.swing.UIManager.LookAndFeelInfo info : javax.swing.UIManager.getInstalledLookAndFeels()) {
                if ("Nimbus".equals(info.getName())) {
                    javax.swing.UIManager.setLookAndFeel(info.getClassName());
                    break;
                }
            }
        } catch (ClassNotFoundException ex) {
            java.util.logging.Logger.getLogger(Swarm.class.getName()).log(java.util.logging.Level.SEVERE, null, ex);
        } catch (InstantiationException ex) {
            java.util.logging.Logger.getLogger(Swarm.class.getName()).log(java.util.logging.Level.SEVERE, null, ex);
        } catch (IllegalAccessException ex) {
            java.util.logging.Logger.getLogger(Swarm.class.getName()).log(java.util.logging.Level.SEVERE, null, ex);
        } catch (javax.swing.UnsupportedLookAndFeelException ex) {
            java.util.logging.Logger.getLogger(Swarm.class.getName()).log(java.util.logging.Level.SEVERE, null, ex);
        }
        //</editor-fold>

        /* Create and display the form */
        java.awt.EventQueue.invokeLater(new Runnable() {
            public void run() {
                new Swarm().setVisible(true);
            }
        });
    }

    // Variables declaration - do not modify//GEN-BEGIN:variables
    private javax.swing.JPanel CustomPanel;
    // End of variables declaration//GEN-END:variables
}
