#include "math_utils.h"
#include "vex.h"

int Math_Utils::normalize_angle(int angle) {
    angle = angle % 360;
    if(angle > 180) {
        angle -= 360;
    } else if(angle < -180) {
        angle += 360;
    }
    return angle;
}

double Math_Utils::calculate_optimal_turn(double current_angle, double target_angle) {
    current_angle = mod(current_angle, 360.0);
    target_angle = mod(target_angle, 360.0);

    double anglular_difference = target_angle - current_angle;
    if(anglular_difference > 180) {
        anglular_difference -= 360;
    } else if(anglular_difference < -180) {
        anglular_difference += 360;
    }

    return anglular_difference;
}

double Math_Utils::circleIntersectionX(double point[2], double otherPoint[2], double r, double d, int direction) {
    double a = 0.5 * (point[0] + otherPoint[0]);
    double b = direction * 0.5 * sqrt((4 * pow(r, 2) / pow(d, 2)) - 1);
    return a + b * (otherPoint[1] - point[1]);
}
double Math_Utils::circleIntersectionY(double point[2], double otherPoint[2], double r, double d, int direction) {
    double a = 0.5 * (point[1] + otherPoint[1]);
    double b = direction * 0.5 * sqrt((4 * pow(r, 2) / pow(d, 2)) - 1);
    return a + b * (point[0] - otherPoint[0]);
}

//calculatePercentDifference: determines the difference in percentage from the average percentage of distance travelled
double Math_Utils::calculatePercentDifference(double totalDistance, double calcDistance, double otherDistance) {
        double calcPercent = calcDistance / totalDistance;
        double otherPercent = otherDistance / totalDistance;
        double avgPercent = (calcPercent + otherPercent) / 2.0;
        return (avgPercent - calcPercent) * 100.0;
}

double Math_Utils::max(double x, double y) {
  if (x >= y) {
    return x;
  } else {
    return y;
  }
}

double Math_Utils::curve(double x) {
  return (0.1 * x + 0.9 * pow(x, 3));
}

double Math_Utils::mod(double a, double b)
{
    double mod;
    // Handling negative values
    if (a < 0)
        mod = -a;
    else
        mod =  a;
    if (b < 0)
        b = -b;

    // Finding mod by repeated subtraction
    
    while (mod >= b)
        mod = mod - b;

    // Sign of result typically depends
    // on sign of a.
    if (a < 0)
        return -mod;

    return mod;
}