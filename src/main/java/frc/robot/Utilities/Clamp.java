package frc.robot.Utilities;

public class Clamp {
    public static int clamp(int x, int min, int max) {
        if (x > max) {
            return max;
        } else if (x < min) {
            return min;
        }
        return x;
    }
    public static double clamp(double x, double min, double max) {
        if (x > max) {
            return max;
        } else if (x < min) {
            return min;
        }
        return x;
    }
}
