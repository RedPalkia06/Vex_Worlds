#ifndef AUTONOMOUS_H
#define AUTONOMOUS_H

double calculatePercentDifference(double totalDistance, double calcDistance, double otherDistance);
bool drive_for(brain Brain, Drivetrain drivetrain, double distance, double velocity, double timeout);


#endif