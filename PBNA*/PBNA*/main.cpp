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
 *  use of multithreading on C++. Due to time restrictions
 *  solution is limited to 2 agents, but it can be amplified.
 *
 *  Program driver.
 */


#include <chrono>
#include <queue>
#include <map>
#include <list>
#include <algorithm>
#include <thread>
#include <mutex>
#include <iostream>
#include <stack>
#include <cfloat>
#include <set>
#include "heuristics.h"
#include "helper.h"

using namespace std;
using namespace std::chrono;

#define ROW 15
#define COL 15

/* Description of the Grid:
0--> The cell is not blocked
1--> The cell is blocked */
int explorationSpace[ROW][COL] =
{
    {0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0},
    {0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0},
    {0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0},
    {0, 1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 1, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0},
    {1, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
    {0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0},
    {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0},
    {0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0},
    {0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1},
    {0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0},
    {0, 0, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 0, 1, 0},
    {0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0},

};

// Mutex for structure blocking
mutex vector_mutex;

// Shared map structure
vector<pair<pair<int, int>, list<agent>>> sharedVector;

// Function to update the shared structure
void updateSharedVector(pair<int, int> coordinates, agent currAgent){
    // The mutex is automatically released when lock goes out of scope
    const lock_guard<mutex> lock(vector_mutex);
    
    bool foundFlag = false;
    for(int i = 0; i < sharedVector.size(); i++){
        if(sharedVector.at(i).first.first == coordinates.first && sharedVector.at(i).first.second == coordinates.second){
            sharedVector.at(i).second.push_back(currAgent);
            foundFlag = true;
        }
    }
    
    if(!foundFlag){
        list<agent> agents;
        agents.push_back(currAgent);
        sharedVector.push_back(make_pair(coordinates, agents));
    }
}

// Function for tracing the path from the source to destination
void tracePath(cell cellDetails[ROW][COL], Pair dest, agent currAgent)
{
    int row = dest.first;
    int col = dest.second;

    stack<Pair> Path;

    while (!(cellDetails[row][col].parent_i == row && cellDetails[row][col].parent_j == col))
    {
        Path.push (make_pair (row, col));
        int temp_row = cellDetails[row][col].parent_i;
        int temp_col = cellDetails[row][col].parent_j;
        row = temp_row;
        col = temp_col;
    }

    Path.push (make_pair (row, col));
    while (!Path.empty())
    {
        pair<int,int> p = Path.top();
        Path.pop();
        updateSharedVector(p, currAgent);
    }
    
    return;
}

// Performing A* search algorithm
void aStarSearch(Pair src, Pair dest, agent currAgent)
{
    // If the source is out of range
    if (isValid (src.first, src.second) == false)
    {
        printf ("Source is invalid\n");
        return;
    }

    // If the destination is out of range
    if (isValid (dest.first, dest.second) == false)
    {
        printf ("Destination is invalid\n");
        return;
    }

    // If the destination cell is the same as source cell
    if (isDestination(src.first, src.second, dest) == true)
    {
        printf ("We are already at the destination\n");
        return;
    }

    // Create a closed list
    bool closedList[ROW][COL];
    
    // Fill it with false, since no cell has been included yet
    memset(closedList, false, sizeof (closedList));

    // Declare a 2D array of structure to hold the cell details
    cell cellDetails[ROW][COL];

    int i, j;

    for (i=0; i<ROW; i++)
    {
        for (j=0; j<COL; j++)
        {
            cellDetails[i][j].parent_i = -1;
            cellDetails[i][j].parent_j = -1;
            cellDetails[i][j].f = FLT_MAX;
            cellDetails[i][j].g = FLT_MAX;
            cellDetails[i][j].h = FLT_MAX;
        }
    }

    // Initialising the parameters of the starting node
    i = src.first;
    j = src.second;
    cellDetails[i][j].parent_i = i;
    cellDetails[i][j].parent_j = j;
    cellDetails[i][j].f = 0.0;
    cellDetails[i][j].g = 0.0;
    cellDetails[i][j].h = 0.0;

    /*
    Create an open list having information as-
    <f, <i, j>>
    where f = g + h,
    and i, j are the row and column index of that cell.
    This open list is implemented as a set of pair of pair.
    Sets will always mantain elements in order, sorting first
    by the f value and then by i and j, ascendingly.
     */
    set<pPair> openList;

    // Put the starting cell on the open list and set its 'f' as 0
    openList.insert(make_pair (0.0, make_pair (i, j)));

    // We set this boolean value as false as destination is not reached
    bool foundDest = false;

    while (!openList.empty())
    {
        pPair p = *openList.begin();

        // Remove this vertex from the open list
        openList.erase(openList.begin());

        // Add this vertex to the closed list
        i = p.second.first;
        j = p.second.second;
        closedList[i][j] = true;
    
      /*
        Generating all the 4 successor of this cell

                   N [i-1][j]
                   |
    [i][j-1] W -- Cell [i][j] -- E [i][j+1]
                   |
                   S [i+1][j]

        Cell-->Popped Cell (i, j)
        N --> North    (i-1, j)
        S --> South    (i+1, j)
        E --> East     (i, j+1)
        W --> West     (i, j-1)
       */

        // To store the 'g', 'h' and 'f' of the 4 successors
        double gNew, hNew, fNew;

        //----------- 1st Successor (North) ------------

        // Only process this cell if this is a valid one
        if (isValid(i-1, j))
        {
            // If the destination cell is the same as the successor
            if (isDestination(i-1, j, dest))
            {
                // Set the oarent of the destination cell
                cellDetails[i-1][j].parent_i = i;
                cellDetails[i-1][j].parent_j = j;
                tracePath (cellDetails, dest, currAgent);
                foundDest = true;
                return;
            }
            // If the successor is already on the closed or blocked, ignore.
            else if (closedList[i-1][j] == false && isUnBlocked(i-1, j, explorationSpace) == true)
            {
                // Adding one to the cost, since I am moving 1 pos
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateManhattanValue(i - 1, j, dest.first, dest.second);
                fNew = gNew + hNew;

                // If it isn’t on the open list, add it
                // If the new cost is lesser than past cost, add it too, since we have found a better path
                if (cellDetails[i-1][j].f == FLT_MAX || cellDetails[i-1][j].f > fNew)
                {
                    openList.insert(make_pair(fNew, make_pair(i-1, j)));
                    // Update the details of this cell
                    cellDetails[i-1][j].f = fNew;
                    cellDetails[i-1][j].g = gNew;
                    cellDetails[i-1][j].h = hNew;
                    cellDetails[i-1][j].parent_i = i;
                    cellDetails[i-1][j].parent_j = j;
                }
            }
        }

        //----------- 2nd Successor (South) ------------

        if (isValid(i+1, j))
        {
            if (isDestination(i+1, j, dest))
            {
                cellDetails[i+1][j].parent_i = i;
                cellDetails[i+1][j].parent_j = j;
                tracePath (cellDetails, dest, currAgent);
                foundDest = true;
                return;
            }
            else if (closedList[i+1][j] == false && isUnBlocked(i+1, j, explorationSpace) == true)
            {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateManhattanValue(i + 1, j, dest.first, dest.second); // Manhattan distance call
                fNew = gNew + hNew;

                if (cellDetails[i+1][j].f == FLT_MAX || cellDetails[i+1][j].f > fNew)
                {
                    openList.insert(make_pair(fNew, make_pair(i+1, j)));
                    cellDetails[i+1][j].f = fNew;
                    cellDetails[i+1][j].g = gNew;
                    cellDetails[i+1][j].h = hNew;
                    cellDetails[i+1][j].parent_i = i;
                    cellDetails[i+1][j].parent_j = j;
                }
            }
        }

        //----------- 3rd Successor (East) ------------

        if (isValid (i, j+1))
        {
            if (isDestination(i, j+1, dest))
            {
                cellDetails[i][j+1].parent_i = i;
                cellDetails[i][j+1].parent_j = j;
                tracePath (cellDetails, dest, currAgent);
                foundDest = true;
                return;
            }
            else if (closedList[i][j+1] == false && isUnBlocked (i, j+1, explorationSpace) == true)
            {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateManhattanValue(i, j + 1, dest.first, dest.second); // Manhattan distance call
                fNew = gNew + hNew;

                if (cellDetails[i][j+1].f == FLT_MAX || cellDetails[i][j+1].f > fNew)
                {
                    openList.insert(make_pair(fNew, make_pair(i, j+1)));
                    cellDetails[i][j+1].f = fNew;
                    cellDetails[i][j+1].g = gNew;
                    cellDetails[i][j+1].h = hNew;
                    cellDetails[i][j+1].parent_i = i;
                    cellDetails[i][j+1].parent_j = j;
                }
            }
        }

        //----------- 4th Successor (West) ------------

        if (isValid(i, j-1))
        {
            if (isDestination(i, j-1, dest))
            {
                cellDetails[i][j-1].parent_i = i;
                cellDetails[i][j-1].parent_j = j;
                tracePath (cellDetails, dest, currAgent);
                foundDest = true;
                return;
            }
            else if (closedList[i][j-1] == false && isUnBlocked(i, j-1, explorationSpace) == true)
            {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateManhattanValue(i, j - 1, dest.first, dest.second); // Manhattan distance call
                fNew = gNew + hNew;

                if (cellDetails[i][j-1].f == FLT_MAX || cellDetails[i][j-1].f > fNew)
                {
                    openList.insert(make_pair (fNew, make_pair (i, j-1)));
                    cellDetails[i][j-1].f = fNew;
                    cellDetails[i][j-1].g = gNew;
                    cellDetails[i][j-1].h = hNew;
                    cellDetails[i][j-1].parent_i = i;
                    cellDetails[i][j-1].parent_j = j;
                }
            }
        }
    }

    // When the destination cell is not found and the open list is empty, we did not reach the cell.
    if (foundDest == false){
        cout << "Failed to find destination cell" << endl;
    }
    return;
}

// Cleans the occurences of determined agent on a grid
void removeAgentOccupied(int agentId, Pair srcNode, Pair destNode){
    for(int i = 0; i < ROW; i++){
        for(int j = 0; j < COL; j++){
            if((srcNode.first == i && srcNode.second == j) || (destNode.first == i && srcNode.second == j)){
                continue;
            }else{
                if (explorationSpace[i][j] == agentId) {
                    explorationSpace[i][j] = 0;
                }
            }
        }
    }
}

Pair backtrack(Pair source, int agentId){
    bool foundStartCoordinates = false;
    Pair newCoordinates = make_pair(-1, -1);
    for(int i = 0; i < sharedVector.size(); i++){
        // If we have not found starting point but already collided, lets quit
        if(!foundStartCoordinates && sharedVector.at(i).second.size()){
            return newCoordinates;
        }
        // 1st step: Find beginning point on the shared map structure
        if(sharedVector.at(i).first.first == source.first && sharedVector.at(i).first.second == source.second){
            foundStartCoordinates = true;
        }
        // 2nd step: Once we have found start coordinates, we can start to backtrack
        if(foundStartCoordinates){
            if(sharedVector.at(i).second.size() == 1){
                cout << "Bactracked to coordinates " << sharedVector.at(i).first.first << " " << sharedVector.at(i).first.second << endl;
                newCoordinates = sharedVector.at(i).first;
            }else if(sharedVector.at(i).second.size() > 1){
                break;
            }
        }
    }
    cout << endl;
    // 3rd step: Return updated coordinates
    return newCoordinates;
}


// Driver program to test multiple agent A*
int main()
{
    // Provide the initial grid configuration
    printGrid(explorationSpace);

    // Capture data for both agents
    int x1, y1, x2, y2, p1;

    while (true) {
        cout << "Please give me the start point for first agent: " << endl;
        cin >> x1 >> y1;
        cout << "Please give me the end point for first agent: " << endl;
        cin >> x2 >> y2;
        if(x1 == x2 && y1 == y2){
            cout << "Start and end point cannot be the same." << endl;
        } else if((!isValid(x1, y1) || !isValid(x2, y2)) || (!isUnBlocked(x1, y1, explorationSpace) || !isUnBlocked(x2, y2, explorationSpace))) {
            cout << "Given point(s) is blocked or not valid, try again." << endl;
        } else {
            break;
        }
    }
    
    while (true) {
        cout << "Please give me the priority for first agent (1 or 2): " << endl;
        cin >> p1;
        if(p1 != 1 && p1 != 2){
            cout << "Priority goes from 1 to 2; 1 having greater priority. Try again." << endl;
        }else{
            break;
        }
    }
    
    Pair src1 = make_pair(x1, y1);

    Pair dest1 = make_pair(x2, y2);
    
    int x11, y11, x22, y22, p2;
    
    while (true) {
        cout << "Please give me the start point for second agent: " << endl;
        cin >> x11 >> y11;
        cout << "Please give me the end point for second agent: " << endl;
        cin >> x22 >> y22;
        if(x11 == x22 && y11 == y22){
            cout << "Start and end point cannot be the same." << endl;
        }else if((!isValid(x11, y11) || !isValid(x22, y22)) || (!isUnBlocked(x11, y11, explorationSpace) || !isUnBlocked(x22, y22, explorationSpace)) || ((x1 == x11 && y1 == y11) || (x1 == x22 && y1 == y22) || (x2 == x11 && y2 == y11) || (x2 == x22 && y2 == y22))) {
            cout << "Given point(s) is blocked or not valid, try again." << endl;
        }else{
            break;
        }
    }
    
    // Having a lower number on the priority, means they are going to be solved first
    while (true) {
        cout << "Please give me the priority for second agent (1 or 2): " << endl;
        cin >> p2;
        if(p2 != 1 && p2 != 2){
            cout << "Priority goes from 1 to 2; 1 having greater priority. Try again." << endl;
        }else if(p1 == p2){
            cout << "Priorities for all agents must be different, try again." << endl;
        }
        else{
            break;
        }
    }
    
    // This identifiers are just to display and identify final routes for agents
    int id1, id2;
    
    while (true) {
        cout << "Finally, give me an identifier for both agents: " << endl;
        cin >> id1 >> id2;
        if(id1 == id2 || (id1 == 0 || id1 == 1 || id2 == 0 || id2 == 1)){
            cout << "Identifiers must be different from each other, and 1 and 0, try again." << endl;
        }else{
            break;
        }
    }
    
    // The static word works for the backtracking
    static Pair src2 = make_pair(x11, y11);

    static Pair dest2 = make_pair(x22, y22);
    
    // Declare agents
    agent a1 = {
       id1,
       p1,
       false
    };
   
    agent a2 = {
       id2,
       p2,
       false
    };
    
    // Place start and end points for agents
    explorationSpace[src1.first][src1.second] = a1.id;
    explorationSpace[src2.first][src2.second] = a2.id;
    
    explorationSpace[dest1.first][dest1.second] = a1.id;
    explorationSpace[dest2.first][dest2.second] = a2.id;
                
    // While agents still need to reach their goal
    while (!a1.hasCompleted || !a2.hasCompleted) {
            bool a1Started = false, a2Started = false;
        
            if(!a1.hasCompleted && !a2.hasCompleted){
                thread th1(aStarSearch, src1, dest1, a1);
                thread th2(aStarSearch, src2, dest2, a2);
                // Speed improvement is here, the optimistic assumption makes it
                // faster compared than running it sequentally for each agent
                th1.join();
                th2.join();
            } else if (!a1.hasCompleted) {
                a2Started  = true;
                cout << "Agent 1 has not finished... wait..." << endl << endl;
                thread th1(aStarSearch, src1, dest1, a1);
                th1.join();
            } else if(!a2.hasCompleted) {
                a1Started = true;
                cout << "Agent 2 has not finished... wait..." << endl << endl;
                thread th2(aStarSearch, src2, dest2, a2);
                th2.join();
            }
                
            bool collisionFound = false, a1Blocked = false, a2Blocked = false;
            if(sharedVector.size() > 0){
                // Check for collisions and update map
                for(auto elem : sharedVector)
                {
                    if(elem.second.size() == 1){
                        if(elem.second.front().id == a1.id && !a1Blocked){
                            if(elem.first.first == dest1.first && elem.first.second == dest1.second){
                                a1.hasCompleted = true;
                            }
                            explorationSpace[elem.first.first][elem.first.second] = a1.id;
                            a1Started = true;
                        }
                        if(elem.second.front().id == a2.id && !a2Blocked){
                            if(elem.first.first == dest2.first && elem.first.second == dest2.second){
                                a2.hasCompleted = true;
                            }
                            explorationSpace[elem.first.first][elem.first.second] = a2.id;
                            a2Started = true;
                            
                        }
                    } else {
                        collisionFound = true;
                        agent first = elem.second.front();
                        elem.second.pop_front();
                        agent second = elem.second.front();
                        if(first.priority < second.priority){
                            explorationSpace[elem.first.first][elem.first.second] = first.id;
                            if(first.id == a1.id){
                                removeAgentOccupied(a2.id, src2, dest2);
                                a2Blocked = true;
                                a1.hasCompleted = true;
                            }else{
                                removeAgentOccupied(a1.id, src1, dest1);
                                a1Blocked = true;
                                a2.hasCompleted = true;
                            }
                        }else{
                            explorationSpace[elem.first.first][elem.first.second] = second.id;
                            if(second.id == a1.id){
                                removeAgentOccupied(a2.id, src2, dest2);
                                a2Blocked = true;
                                a1.hasCompleted = true;
                            }else{
                                removeAgentOccupied(a1.id, src1, dest1);
                                a1Blocked = true;
                                a2.hasCompleted = true;
                            }
                        }
                    }
                }
                
                // Mark all agents as arrived! This could actually happen at the first run
                if(!collisionFound && a1Started && a2Started){
                    cout << "All agents been placed!" << endl;
                    a1.hasCompleted = true;
                    a2.hasCompleted = true;
                }else{
                    // State who has finished
                    if(a1.hasCompleted){
                        cout << "Agent 1 has completed" << endl;
                    }else if(a2.hasCompleted){
                        cout << "Agent 2 has completed" << endl;
                    }
                    // Perform backtrack, that also, saves computing time by advancing the start node
                    if(!a1.hasCompleted){
                        Pair aux = backtrack(src1, a1.id);
                        if(aux.first != -1 && aux.second != -1){
                            src1 = aux;
                        }
                    }else if(!a2.hasCompleted){
                        Pair aux = backtrack(src2, a2.id);
                        if(aux.first != -1 && aux.second != -1){
                            src2 = aux;
                        }
                    }
                }
                
                // Clear the shared DS
                sharedVector.clear();
                // Print the new configuration
                printGrid(explorationSpace);
            }else{
                cout << "Not all agents could be routed" << endl << endl;
                // Print the final configuration if so
                printGrid(explorationSpace);
                break;
            }
        }
    return 0;
}
