package frc.robot.utils;

public class Clamp {
    public static int clamp(int x, int min, int max) {
        if (x > max) {
            return max;
        } else if (x < min) {
            return min;
        }
        return x;
    }
}
