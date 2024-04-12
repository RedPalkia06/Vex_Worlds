#ifndef AUTONOMOUS_H
#define AUTONOMOUS_H

double calculatePercentDifference(double totalDistance, double calcDistance, double otherDistance, double rightDistance);
bool drive_for(brain Brain, Drivetrain drivetrain, double distance, double velocity, double timeout);
bool turn_to(Drivetrain drivetrain, double angle, double velocity);
bool arc_to_point(brain Brain, Drivetrain drivetrain, double initialPoint[2], double finalPoint[2], double radius, double velocity, bool direction, double timeout);

#endif