import java.util.ArrayList;

public class TrajectoryGeneration {
    public static double DT = 0.02;

    public static void main(String args[]) {
        PointsPath pointsPath = new PointsPath();
        String selected = "center_to_right_switch";

        switch(selected) {
            case "center_to_right_switch":
                pointsPath.addPoint(new Point(0, 0, Math.toRadians(0)));
                pointsPath.addPoint(new Point(3, -1.2, Math.toRadians(0)));
                break;

            case "center_to_left_switch":
                pointsPath.addPoint(new Point(0, 0, Math.toRadians(0)));
                pointsPath.addPoint(new Point(3, 2.3, Math.toRadians(0)));

                break;


            case "center_to_right_switch_outside":
                pointsPath.addPoint(new Point(0, 0, Math.toRadians(0)));
                pointsPath.addPoint(new Point(0.8, -2, Math.toRadians(-89)));
                pointsPath.addPoint(new Point(1.25, -2.5, Math.toRadians(-30)));
                pointsPath.addPoint(new Point(3.5, -2.2, Math.toRadians(50)));
                break;


            case "crazy_circle":
                pointsPath.addPoint(new Point(0, 0, Math.toRadians(0)));
                pointsPath.addPoint(new Point(1, -1, Math.toRadians(89)));
                pointsPath.addPoint(new Point(0, -2, Math.toRadians(0)));
                pointsPath.addPoint(new Point(-1, -1, Math.toRadians(-89)));
                pointsPath.addPoint(new Point(0, 0, Math.toRadians(0)));
                break;


            case "right_to_right_scale":
                pointsPath.addPoint(new Point(0, 0, Math.toRadians(0)));
                pointsPath.addPoint(new Point(0.5, 0, Math.toRadians(0)));
                pointsPath.addPoint(new Point(5.263, 0, Math.toRadians(0)));
                pointsPath.addPoint(new Point(7, 0.65, Math.toRadians(0)));
                break;


            case "left_to_left_scale":
                pointsPath.addPoint(new Point(0, 0, Math.toRadians(0)));
                pointsPath.addPoint(new Point(0.5, 0, Math.toRadians(0)));
                pointsPath.addPoint(new Point(5.263, 0, Math.toRadians(0)));
                pointsPath.addPoint(new Point(7.0, -0.65, Math.toRadians(-20)));
                break;


            case "center_to_right_scale":
                pointsPath.addPoint(new Point(0, 0, Math.toRadians(0)));
                pointsPath.addPoint(new Point(0.1, 0, Math.toRadians(0)));
                pointsPath.addPoint(new Point(1.5, -1.8, Math.toRadians(-45)));
                pointsPath.addPoint(new Point(4, -3.5, Math.toRadians(0)));
                pointsPath.addPoint(new Point(5.7, -1.8, Math.toRadians(0)));
                break;


            case "right_to_right_switch":
                pointsPath.addPoint(new Point(0, 0, Math.toRadians(0)));
                pointsPath.addPoint(new Point(3.5-0.8636, 1.5, Math.toRadians(0))); //0.8636 is robot length
                break;


            case "left_to_left_switch":
                pointsPath.addPoint(new Point(0, 0, Math.toRadians(0)));
                pointsPath.addPoint(new Point(3.15, -1.5, Math.toRadians(0))); //0.8636 is robot length
                break;

            case "test":
                pointsPath.addPoint(new Point(0, 0, Math.toRadians(0)));
                pointsPath.addPoint(new Point(5, 5, Math.toRadians(90)));
                break;

            /*
            case "taco":
                pointsPath.addPoint(new Point(0, 0, Math.toRadians(0)));
                pointsPath.addPoint(new Point(5.269, 0, Math.toRadians(0)));
                pointsPath.addPoint(new Point(5.269 + 0.864 + 0.3, 1, Math.toRadians(89)));
                pointsPath.addPoint(new Point(5.269 + 0.864 + 0.3, 4.13, Math.toRadians(89.99)));
                pointsPath.addPoint(new Point(5.269 + 0.864 + 0.3 - 1.7, 4.13 + 0.9, Math.toRadians(0)));
                break;


            case "right_to_left_switch":

                pointsPath.addPoint(new Point(0, 0, Math.toRadians(0)));
                pointsPath.addPoint(new Point(0.25, 0, Math.toRadians(0)));
                pointsPath.addPoint(new Point(1, 1, Math.toRadians(89.99)));
                pointsPath.addPoint(new Point(0.9, 5, Math.toRadians(89.99)));
                //pointsPath.addPoint(new Point(3, 3, Math.toRadians(89.999999999)));
                //pointsPath.addPoint(new Point(3, 0, Math.toRadians(89.99)));
                //pointsPath.addPoint(new Point(4.5, 2, Math.toRadians(89.99)));
                //pointsPath.addPoint(new Point(2.5, 3.25, Math.toRadians(0)));
                break;
            */

            case "baseline_defense":
                pointsPath.addPoint(new Point(0, 0, Math.toRadians(0)));
                pointsPath.addPoint(new Point(6, 0, Math.toRadians(0)));
                break;

        }

        Trajectory splineTrajectory = SplineGeneration.generateSpline(pointsPath);
        Trajectory[] wheelTrajectories = SplineGeneration.generateWheelTrajectories(splineTrajectory, 0.7);
        //FileInput.serializeSplineTraj(splineTrajectory, selected);
    }

    public static Trajectory generate(double finalPosition, double init_heading, double final_heading) {
        double POSITION = finalPosition;
        double MAX_VELOCITY = 1.15;
        double ACCEL = 2.5;

        //100 = .5 * a * t * t

        double JERK  = Math.pow(ACCEL, 1.5);
        double JOUNCE = Math.pow(JERK, 1.5);
        double CRACKLE = Math.pow(JOUNCE, 1.5);

        double Xpeak[] = {POSITION, MAX_VELOCITY, ACCEL, JERK, JOUNCE, CRACKLE};
        double T[] = {0, 0, 0, 0, 0, 0};

        SCurveGeneration.computePeriods(Xpeak, T);

        ArrayList<Double> values = new ArrayList<>();
        ArrayList<Double> velocity = new ArrayList<>();
        ArrayList<Double> acceleration = new ArrayList<>();
        ArrayList<Double> jerk = new ArrayList<>();
        ArrayList<Double> jounce = new ArrayList<>();
        ArrayList<Double> crackle = new ArrayList<>();

        double lastValueVelo = 0;
        double lastValueAccel = 0;
        double lastValueJerk = 0;
        double lastValueJounce = 0;
        double lastValueCrackle = 0;
        int time_in_dt = 0;

        for (int i = 1; i <= 20000; i++) {
            double point = SCurveGeneration.getSetpoint(Xpeak, T, DT * i);
            double delta = (point - lastValueVelo)/DT;
            double delta_accel = ((delta - lastValueAccel)/DT);
            double delta_jerk = ((delta_accel - lastValueJerk)/DT);
            double delta_jounce = ((delta_jerk - lastValueJounce)/DT);
            double delta_crackle = ((delta_jounce - lastValueCrackle)/DT);
            time_in_dt++;
            values.add(point);
            velocity.add(delta);
            acceleration.add(delta_accel);
            jerk.add(delta_jerk);
            jounce.add(delta_jounce);
            crackle.add(delta_crackle);

            lastValueVelo = point;
            lastValueAccel = delta;
            lastValueJerk = delta_accel;
            lastValueJounce = delta_jerk;
            lastValueCrackle = delta_jounce;

            if(delta == 0) {
                break;
            }
        }

        Trajectory trajectory = new Trajectory(time_in_dt);
        Trajectory.Segment[] segments = new Trajectory.Segment[time_in_dt];
        double total_heading_change = final_heading - init_heading;
        for(int i=0; i<time_in_dt; i++) {
            segments[i] = new Trajectory.Segment();
            segments[i].vel = velocity.get(i);
            segments[i].acc = acceleration.get(i);
            segments[i].jerk = jerk.get(i);
            segments[i].pos = values.get(i);
            segments[i].dt = DT;
            segments[i].heading = init_heading + total_heading_change * (segments[i].pos) / finalPosition;
        }
        trajectory.segments_ = segments;

        return trajectory;
    }
}
