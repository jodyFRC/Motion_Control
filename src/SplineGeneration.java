import javax.swing.*;
import java.util.ArrayList;

public class SplineGeneration {

    double a_;  // ax^5
    double b_;  // + bx^4
    double c_;  // + cx^3
    double d_;  // + dx^2
    double e_;  // + ex

    double y_offset_;
    double x_offset_;
    double knot_distance_;
    double theta_offset_;
    double arc_length_;

    private static boolean almostEqual(double x, double y) {
        return Math.abs(x - y) < 1E-6;
    }

    public static SplineGeneration reticulateSplines(Point start, Point end) {
        double sx = start.x;
        double sy = start.y;
        double sh = start.heading;

        double ex = end.x;
        double ey = end.y;
        double eh = end.heading;

        return reticulateSplines(sx, sy, sh, ex, ey, eh);
    }

    public static SplineGeneration reticulateSplines(double x0, double y0, double theta0,
                                                     double x1, double y1, double theta1) {
        SplineGeneration result = new SplineGeneration();

        result.x_offset_ = x0;
        result.y_offset_ = y0;

        double x1_hat = Math.sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
        if (x1_hat == 0) {
            return null;
        }
        result.knot_distance_ = x1_hat;
        result.theta_offset_ = Math.atan2(y1 - y0, x1 - x0);

        double theta0_hat = ChezyMath.getDifferenceInAngleRadians(result.theta_offset_, theta0);
        double theta1_hat = ChezyMath.getDifferenceInAngleRadians(result.theta_offset_, theta1);

        /*
        if (almostEqual(Math.abs(theta0_hat), Math.PI / 2)
                || almostEqual(Math.abs(theta1_hat), Math.PI / 2)) {
            return null;
        }

        if (Math.abs(ChezyMath.getDifferenceInAngleRadians(theta0_hat,
                theta1_hat))
                >= Math.PI / 2) {
            return null;
        }
        */

        double yp0_hat = Math.tan(theta0_hat);
        double yp1_hat = Math.tan(theta1_hat);

        result.a_ = -(3 * (yp0_hat + yp1_hat)) / (x1_hat * x1_hat * x1_hat * x1_hat);
        result.b_ = (8 * yp0_hat + 7 * yp1_hat) / (x1_hat * x1_hat * x1_hat);
        result.c_ = -(6 * yp0_hat + 4 * yp1_hat) / (x1_hat * x1_hat);
        result.d_ = 0;
        result.e_ = yp0_hat;

        return result;
    }

    public static Trajectory[] generateWheelTrajectories(Trajectory trajectory_input, double robot_width_wheels) {
        Trajectory[] trajectory_output = {trajectory_input.copy(), trajectory_input.copy()};
        Trajectory left = trajectory_output[0];
        Trajectory right = trajectory_output[1];

        for (int i = 0; i < trajectory_input.getNumSegments(); ++i) {
            Trajectory.Segment current = trajectory_input.getSegment(i);
            double cos_angle = Math.cos(current.heading);
            double sin_angle = Math.sin(current.heading);

            Trajectory.Segment s_left = left.getSegment(i);
            s_left.x = current.x - robot_width_wheels / 2 * sin_angle;
            s_left.y = current.y + robot_width_wheels / 2 * cos_angle;
            if (i > 0) {
                double dist = Math.sqrt((s_left.x - left.getSegment(i - 1).x)
                        * (s_left.x - left.getSegment(i - 1).x)
                        + (s_left.y - left.getSegment(i - 1).y)
                        * (s_left.y - left.getSegment(i - 1).y));
                s_left.pos = left.getSegment(i - 1).pos + dist;
                s_left.vel = dist / s_left.dt;
                s_left.acc = (s_left.vel - left.getSegment(i - 1).vel) / s_left.dt;
                s_left.jerk = (s_left.acc - left.getSegment(i - 1).acc) / s_left.dt;
            }

            Trajectory.Segment s_right = right.getSegment(i);
            s_right.x = current.x + robot_width_wheels / 2 * sin_angle;
            s_right.y = current.y - robot_width_wheels / 2 * cos_angle;
            if (i > 0) {
                double dist = Math.sqrt((s_right.x - right.getSegment(i - 1).x)
                        * (s_right.x - right.getSegment(i - 1).x)
                        + (s_right.y - right.getSegment(i - 1).y)
                        * (s_right.y - right.getSegment(i - 1).y));
                s_right.pos = right.getSegment(i - 1).pos + dist;
                s_right.vel = dist / s_right.dt;
                s_right.acc = (s_right.vel - right.getSegment(i - 1).vel) / s_right.dt;
                s_right.jerk = (s_right.acc - right.getSegment(i - 1).acc) / s_right.dt;
            }
        }
        ArrayList<Double> leftVelocity = new ArrayList<>();
        ArrayList<Double> rightVelocity = new ArrayList<>();
        ArrayList<Double> traj = new ArrayList<>();

        for (int i = 0; i < trajectory_output[0].segments_.length; i++) {

            double l_v = trajectory_output[0].segments_[i].vel;
            double r_v = trajectory_output[1].segments_[i].vel;

            leftVelocity.add(l_v);
            rightVelocity.add(r_v);
            traj.add(trajectory_input.segments_[i].y);
            //System.out.println("{" + l_v + "," + r_v + "},");/
            System.out.println(trajectory_input.segments_[i].x);
        }

        SwingUtilities.invokeLater(new Runnable() {
            public void run() {
                GraphUtil.createAndShowGui(leftVelocity, "L");
                GraphUtil.createAndShowGui(rightVelocity, "R");
                GraphUtil.createAndShowGui(traj, "TRAJ");
            }
        });

        Graph g = new Graph(trajectory_input);

        System.out.println("Time: " + traj.size() * TrajectoryGeneration.DT);

        return new Trajectory[]{trajectory_output[0], trajectory_output[1]};
    }

    public double calculateLength() {
        if (arc_length_ >= 0.00001) {
            return arc_length_;
        }
        final int kNumSamples = 100000;
        double arc_length = 0;
        double t, dydt;
        double integrand, last_integrand
                = Math.sqrt(1 + derivativeAt(0) * derivativeAt(0)) / kNumSamples;
        for (int i = 1; i <= kNumSamples; ++i) {
            t = ((double) i) / kNumSamples;
            dydt = derivativeAt(t);
            integrand = Math.sqrt(1 + dydt * dydt) / kNumSamples;
            arc_length += (integrand + last_integrand) / 2;
            last_integrand = integrand;
        }
        arc_length_ = knot_distance_ * arc_length;
        return arc_length_;
    }

    public double getPercentageForDistance(double distance) {
        final int kNumSamples = 100000;
        double arc_length = 0;
        double t = 0;
        double last_arc_length = 0;
        double dydt;
        double integrand, last_integrand
                = Math.sqrt(1 + derivativeAt(0) * derivativeAt(0)) / kNumSamples;
        distance /= knot_distance_;
        for (int i = 1; i <= kNumSamples; ++i) {
            t = ((double) i) / kNumSamples;
            dydt = derivativeAt(t);
            integrand = Math.sqrt(1 + dydt * dydt) / kNumSamples;
            arc_length += (integrand + last_integrand) / 2;
            if (arc_length > distance) {
                break;
            }
            last_integrand = integrand;
            last_arc_length = arc_length;
        }

        double interpolated = t;
        if (arc_length != last_arc_length) {
            interpolated += ((distance - last_arc_length)
                    / (arc_length - last_arc_length) - 1) / (double) kNumSamples;
        }
        return interpolated;
    }

    public double[] getXandY(double percentage) {
        double[] result = new double[2];

        percentage = Math.max(Math.min(percentage, 1), 0);
        double x_hat = percentage * knot_distance_;
        double y_hat = (a_ * x_hat + b_) * x_hat * x_hat * x_hat * x_hat
                + c_ * x_hat * x_hat * x_hat + d_ * x_hat * x_hat + e_ * x_hat;

        double cos_theta = Math.cos(theta_offset_);
        double sin_theta = Math.sin(theta_offset_);

        result[0] = x_hat * cos_theta - y_hat * sin_theta + x_offset_;
        result[1] = x_hat * sin_theta + y_hat * cos_theta + y_offset_;
        return result;
    }

    public double valueAt(double percentage) {
        percentage = Math.max(Math.min(percentage, 1), 0);
        double x_hat = percentage * knot_distance_;
        double y_hat = (a_ * x_hat + b_) * x_hat * x_hat * x_hat * x_hat
                + c_ * x_hat * x_hat * x_hat + d_ * x_hat * x_hat + e_ * x_hat;

        double cos_theta = Math.cos(theta_offset_);
        double sin_theta = Math.sin(theta_offset_);

        double value = x_hat * sin_theta + y_hat * cos_theta + y_offset_;
        return value;
    }

    private double derivativeAt(double percentage) {
        percentage = Math.max(Math.min(percentage, 1), 0);

        double x_hat = percentage * knot_distance_;
        double yp_hat = (5 * a_ * x_hat + 4 * b_) * x_hat * x_hat * x_hat + 3 * c_ * x_hat * x_hat
                + 2 * d_ * x_hat + e_;

        return yp_hat;
    }

    public double angleAt(double percentage) {
        double angle = ChezyMath.boundAngle0to2PiRadians(
                Math.atan(derivativeAt(percentage)) + theta_offset_);
        return angle;
    }

    public String toString() {
        return "a=" + a_ + "; b=" + b_ + "; c=" + c_ + "; d=" + d_ + "; e=" + e_;
    }

    public static Trajectory generateSpline(PointsPath pointsPath) {
        SplineGeneration[] splines = new SplineGeneration[pointsPath.path.size() - 1]; //we need 1 spline for n-1 points
        double total_distance = 0;
        for (int i = 0; i < splines.length; ++i) {
            splines[i] = new SplineGeneration();
            SplineGeneration result = reticulateSplines(pointsPath.path.get(i), pointsPath.path.get(i + 1));
            splines[i] = result;
            if (result == null) {
                return null;
            }
            System.out.println(splines[i].calculateLength());
            total_distance = total_distance + splines[i].calculateLength();
        }

        //System.out.println(total_distance);

        Trajectory traj = TrajectoryGeneration.generate(total_distance, pointsPath.path.get(0).heading, pointsPath.path.get(pointsPath.path.size() - 1).heading);

        //create headings
        int cur_spline = 0;
        double cur_spline_start_pos = 0;
        double length_of_splines_finished = 0;
        for (int i = 0; i < traj.getNumSegments(); ++i) {
            double cur_pos = traj.getSegment(i).pos;

            boolean found_spline = false;
            while (!found_spline) {
                double cur_pos_relative = cur_pos - cur_spline_start_pos;
                if (cur_pos_relative <= splines[cur_spline].calculateLength()) {
                    double percentage = splines[cur_spline].getPercentageForDistance(
                            cur_pos_relative);
                    traj.getSegment(i).heading = splines[cur_spline].angleAt(percentage);
                    double[] coords = splines[cur_spline].getXandY(percentage);
                    traj.getSegment(i).x = coords[0];
                    traj.getSegment(i).y = coords[1];
                    found_spline = true;
                } else if (cur_spline < splines.length - 1) {
                    length_of_splines_finished += splines[cur_spline].calculateLength();
                    cur_spline_start_pos = length_of_splines_finished;
                    ++cur_spline;
                } else {
                    traj.getSegment(i).heading = splines[splines.length - 1].angleAt(1.0);
                    double[] coords = splines[splines.length - 1].getXandY(1.0);
                    traj.getSegment(i).x = coords[0];
                    traj.getSegment(i).y = coords[1];
                    found_spline = true;
                }
            }
        }

        return traj;
    }

}
