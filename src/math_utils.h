#ifndef MATH_UTILS
#define MATH_UTILS

class Math_Utils {
    public:
        static double calculatePercentDifference(double totalDistance, double calcDistance, double otherDistance);
        static double max(double x, double y);
        static double curve(double x);
        static int normalize_angle(int angle);
        static double circleIntersectionX(double point[2], double otherPoint[2], double r, double d, int direction);
        static double circleIntersectionY(double point[2], double otherPoint[2], double r, double d, int direction);
        static int calculate_optimal_turn(int current_angle, int target_angle);
};
#endif