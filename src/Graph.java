import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.File;
import javax.imageio.ImageIO;
import javax.swing.JFrame;
import javax.swing.JPanel;

public class Graph extends JFrame {
    JFrame f = new JFrame();
    JPanel jp;

    Trajectory traj;

    public Graph(Trajectory _traj) {
        traj = _traj;
        f.setTitle("Graph");
        f.setSize(1000, 1000);
        f.setDefaultCloseOperation(EXIT_ON_CLOSE);

        jp = new GPanel();
        f.add(jp);
        f.setVisible(true);
    }

    class GPanel extends JPanel {
        public GPanel() {
            f.setPreferredSize(new Dimension(1000, 1000));
        }

        public void paintComponent(Graphics g) {

            int scale = 85;

            try {
                BufferedImage myPicture = ImageIO.read(new File("C:\\Users\\LGHS Robotics\\Downloads\\BGIiftQ.jpg"));
                Image reImage = myPicture.getScaledInstance(960 * 2, 540 * 2, Image.SCALE_DEFAULT);
                g.drawImage(reImage, 0, 0, null);

                for (int i = 0; i < 1000; i = i + scale) {
                    for (int j = 0; j < 1000; j = j + scale) {
                        g.setColor(Color.gray);
                        g.drawLine(i, j, i, j + scale);
                        g.drawLine(i, j, i + scale, j);
                    }
                }

                g.setColor(Color.black);

                for (int i = 0; i < traj.getNumSegments(); i++) {
                    g.setColor(Color.BLACK);

                    int x = (3*scale) + (int) ((double) traj.getSegment(i).x * scale);
                    int y = (6*scale) + (int) ((double) traj.getSegment(i).y * scale);

                    g.drawRect(x, y, 1, 1);
                    if((i == traj.getNumSegments()-1) || (i % 20000 == 0)) {
                        g.setColor(Color.GREEN);
                        g.drawRect(x, y, (int)((double)scale * 0.7112), (int)((double)scale * 0.8382));
                    }
                }



            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }
}