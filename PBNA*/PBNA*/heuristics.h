/* GNU GENERAL PUBLIC LICENSE

Version 1.0.0, 18 November 2020

Copyright © 2007 Free Software Foundation, Inc. <https://fsf.org/>

Everyone is permitted to copy and distribute verbatim copies of this license document, but changing it is not allowed.
 
 */

/*
 *  Final Project Programming Languages
 *
 *  Created on: November 2020
 *  Author: Alejandro Gleason Méndez
 *  ID: A01703013
 *
 *  This programs acts as a proof of concept for a
 *  problem of multi agent path finding. It makes
 *  use of multithreading on C++.
 *
 *  Library to hold heuristics to calculate h value.
 */

#ifndef heuristics_h
#define heuristics_h

using namespace std;

// Manhattan distance returns "total" cells apart, considering we can move in 4 directions
double calculateManhattanValue(int nodeX, int nodeY, int goalX, int goalY)
{
    double h = abs(nodeX - goalX) + abs(nodeY - goalY);
    return h;
}

// Chebyshev distance is great for adding diagonal movements
int calculateChebyshevDistance(int nodeX, int nodeY, int goalX, int goalY)
{
    int dx = abs(nodeX - goalX);
    int dy = abs(nodeY - goalY);
    return (dx + dy) - min(dx, dy);
}

// Euclidean distance works for movements that can be made at any angle, tracing a straight line
int calculateEuclideanDistance(int nodeX, int nodeY, int goalX, int goalY)
{
    int dx = abs(nodeX - goalX);
    int dy = abs(nodeY - goalY);
    //sqrt computation can be expensive
    return sqrt(dx * dx + dy * dy);
}

#endif /* heuristics_h */
