import java.util.ArrayList;

/**
 * Created by Jody on 10/22/2017.
 */
public class PointsPath {
    public ArrayList<Point> path;
    public PointsPath() {
        path = new ArrayList<>();
    }
    public void addPoint(Point point) {
        path.add(point);
    }
}
