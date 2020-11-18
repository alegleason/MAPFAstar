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
mutex map_mutex;
mutex vector_mutex;

// Creating a shortcut for int, int pair type
typedef pair<int, int> Pair;

// Creating a shortcut for pair<int, pair<int, int>> type
typedef pair<double, pair<int, int>> pPair;

// A structure to hold the neccesary parameters
struct cell
{
    // Row and Column index of its parent
    // Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
    int parent_i, parent_j;
    // f = g + h
    double f, g, h;
};

// Struct that holds the identifier for each agent and its priority
struct agent {
    int id;
    int priority;
    bool hasCompleted;
};

// Shared map structure
vector<pair<pair<int, int>, list<agent>>> sharedVector;


// This is an strucure which implements the
// operator overlading (F should be lesser)
struct CompareF {
    bool operator()(cell const& c1, cell const& c2)
    {
        // return "true" if "p1" is ordered
        // before "p2", for example:
        return c1.f < c2.f;
    }
};

// A Utility Function to check whether given cell (row, col)
// is a valid cell or not.
bool isValid(int row, int col)
{
    // Returns true if row number and column number
    // is in range
    return (row >= 0) && (row < ROW) && (col >= 0) && (col < COL);
}

// A Utility Function to check whether the given cell is
// blocked or not
bool isUnBlocked(int row, int col)
{
    // Returns true if the cell is not blocked else false
    if (explorationSpace[row][col] == 0){
        return true;
    }else{
        return false;
    }
}

// A Utility Function to check whether destination cell has
// been reached or not
bool isDestination(int row, int col, Pair dest)
{
    if (row == dest.first && col == dest.second){
        return true;
    }
    else{
        return false;
    }
}

// A Utility Function to calculate the 'h' heuristics.
double calculateHValue(int row, int col, Pair dest)
{
    // Return using the Manhattan distance, which returns "total" cells apart, considering we can move in 4 directions
    double h = abs(row - dest.first) + abs(col - dest.second);
    // printf("Manhattan value for cell %i %i is %f\n", row, col, h);
    return h;
}

// Function to update the shared structure
void updateSharedVector(pair<int, int> coordinates, agent currAgent){
    // map_mutex is automatically released when lock goes out of scope
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


// A Utility Function to trace the path from the source
// to destination
void tracePath(cell cellDetails[][COL], Pair dest, agent currAgent)
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

// A Function to find the shortest path between
// a given source cell to a destination cell according
// to A* Search Algorithm
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

    // Either the source or the destination is blocked
    /*if (isUnBlocked(src.first, src.second) == false ||
            isUnBlocked(dest.first, dest.second) == false)
    {
        printf ("Source or the destination is blocked\n");
        return;
    }*/

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
    and i, j are the row and column index of that cell
    Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
    This open list is implemented as a set of pair of pair.
    Sets will always mantain elements in order, sorting first
    by the f value and then by i and j, ascendingly.*/
    set<pPair> openList;

    // Put the starting cell on the open list and set its
    // 'f' as 0, since we have not moved
    openList.insert(make_pair (0.0, make_pair (i, j)));

    // We set this boolean value as false as initially
    // the destination is not reached.
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
            // If the destination cell is the same as the
            // current successor
            if (isDestination(i-1, j, dest))
            {
                // Set the Parent of the destination cell
                cellDetails[i-1][j].parent_i = i;
                cellDetails[i-1][j].parent_j = j;
                tracePath (cellDetails, dest, currAgent);
                foundDest = true;
                return;
            }
            // If the successor is already on the closed
            // list or if it is blocked, then ignore it.
            else if (closedList[i-1][j] == false && isUnBlocked(i-1, j) == true)
            {
                gNew = cellDetails[i][j].g + 1.0; // Adding one to the cost, since I am moving 1 pos
                hNew = calculateHValue (i-1, j, dest); // Manhattan distance call
                fNew = gNew + hNew;

                // If it isn’t on the open list, add it to
                // the open list. Or, if it is on the open list already, check
                // to see if this path to that square is better,
                // using 'f' cost as the measure.
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
            else if (closedList[i+1][j] == false && isUnBlocked(i+1, j) == true)
            {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateHValue(i+1, j, dest);
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
            else if (closedList[i][j+1] == false && isUnBlocked (i, j+1) == true)
            {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateHValue (i, j+1, dest);
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
            else if (closedList[i][j-1] == false && isUnBlocked(i, j-1) == true)
            {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateHValue(i, j-1, dest);
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

    // When the destination cell is not found and the open
    // list is empty, means we were not able to reach the cell.
    if (foundDest == false){
        cout << "Failed to find destination cell" << endl;
    }
    
    return;
}

// Cleans the occurences of determined agent on a grid
void removeAgentOccupied(int agentId){ // TODO: Leave start and end point!
    for(int i = 0; i < ROW; i++){
        for(int j = 0; j < COL; j++){
            if (explorationSpace[i][j] == agentId) {
                explorationSpace[i][j] = 0;
            }
        }
    }
}

// Prints the content of the shared structure
void printSharedMapVector(){
    cout << "Vector content: " << endl;
    for(int i = 0; i < sharedVector.size(); i++){
        cout << "coordinates: " << sharedVector.at(i).first.first << " " <<  sharedVector.at(i).first.second << endl << "agents: " << endl;
        for (auto const& agent:  sharedVector.at(i).second){
            cout << agent.id << " ";
        }
        cout << endl;
    }
}

// Function that prints current exploration space
void printGrid(){
    cout << "\nEXPLORATION SPACE " << endl;
    for(int i = 0; i < ROW; i++){
        for(int j = 0; j < COL; j++){
            cout << explorationSpace[i][j] << " ";
        }
        cout << endl;
    }
    cout << endl;
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
    printGrid();

    // Capture relevant data for both agents
    
    int x1, y1, x2, y2, p1;

    while (true) {
        cout << "Please give me the start point for first agent: " << endl;
        cin >> x1 >> y1;
        cout << "Please give me the end point for first agent: " << endl;
        cin >> x2 >> y2;
        if(x1 == x2 && y1 == y2){
            cout << "Start and end point cannot be the same." << endl;
        } else if((!isValid(x1, y1) || !isValid(x2, y2)) || (!isUnBlocked(x1, y1) && !isUnBlocked(x2, y2))) {
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
        }else if((!isValid(x11, y11) || !isValid(x22, y22)) || (!isUnBlocked(x11, y11) && !isUnBlocked(x22, y22)) || ((x1 == x11 && y1 == y11) || (x1 == x22 && y1 == y22) || (x2 == x11 && y2 == y11) || (x2 == x22 && y2 == y22))) {
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
    
    int id1, id2;
    
    // This identifiers are just to display and identify final routes for agents
    
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
    
    cout << "Grid with agents placed: " << endl;
    
    explorationSpace[src1.first][src1.second] = a1.id;
    explorationSpace[src2.first][src2.second] = a2.id;
    
    explorationSpace[dest1.first][dest1.second] = a1.id;
    explorationSpace[dest2.first][dest2.second] = a2.id;
    
    printGrid();
            
    // While agents still need to reach their goal
    while (!a1.hasCompleted || !a2.hasCompleted) {
            bool a1Started = false, a2Started = false;
        
            if(!a1.hasCompleted && !a2.hasCompleted){
                cout << "starting agent " << a1.id << " with coordinates: " << src1.first << " " << src1.second << endl;
                cout << "starting agent " << a2.id << " with coordinates: " << src2.first << " " << src2.second << endl;
                thread th1(aStarSearch, src1, dest1, a1);
                thread th2(aStarSearch, src2, dest2, a2);
                // Speed improvement is here, the optimistic assumption makes it
                // faster compared than running it sequentally for each agent
                th1.join();
                th2.join();
            } else if (!a1.hasCompleted) {
                a2Started  = true;
                cout << "Agent 1 has not finished... wait..." << endl << endl;
                cout << "starting agent " << a1.id << " with coordinates: " << src1.first << " " << src1.second << endl;
                thread th1(aStarSearch, src1, dest1, a1);
                th1.join();
            } else if(!a2.hasCompleted) {
                a1Started = true;
                cout << "Agent 2 has not finished... wait..." << endl << endl;
                cout << "starting agent " << a2.id << " with coordinates: " << src2.first << " " << src2.second << endl;
                thread th2(aStarSearch, src2, dest2, a2);
                th2.join();
            }
        
            printSharedMapVector();
        
            bool collisionFound = false, a1Blocked = false, a2Blocked = false;
            if(sharedVector.size() > 0){
                // Check for collisions and update map
                for(auto elem : sharedVector)
                {
                    if(elem.second.size() == 1){
                        if(elem.second.front().id == a1.id && !a1Blocked){
                            if(elem.first.first == dest1.first && elem.first.second == dest1.second){
                                cout << "reached goal for agent 1" << endl;
                                a1.hasCompleted = true;
                            }
                            explorationSpace[elem.first.first][elem.first.second] = a1.id;
                            a1Started = true;
                        }
                        if(elem.second.front().id == a2.id && !a2Blocked){
                            if(elem.first.first == dest2.first && elem.first.second == dest2.second){
                                cout << "reached goal for agent 2" << endl;
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
                            removeAgentOccupied(second.id);
                            if(first.id == a1.id){
                                a2Blocked = true;
                                cout << "Agent 1 completed " << endl;
                                a1.hasCompleted = true;
                            }else{
                                a1Blocked = true;
                                cout << "Agent 2 completed " << endl;
                                a2.hasCompleted = true;
                            }
                        }else{
                            explorationSpace[elem.first.first][elem.first.second] = second.id;
                            removeAgentOccupied(first.id);
                            if(second.id == a1.id){
                                a2Blocked = true;
                                cout << "Agent 1 completed " << endl;
                                a1.hasCompleted = true;
                            }else{
                                a1Blocked = true;
                                cout << "Agent 2 completed " << endl;
                                a2.hasCompleted = true;
                            }
                        }
                    }
                }
                
                // Mark all agents as arrived! This could actually happen at the first run, thus, saves time
                if(!collisionFound && a1Started && a2Started){
                    cout << "All agents been placed!" << endl;
                    a1.hasCompleted = true;
                    a2.hasCompleted = true;
                }else{
                    // Perform backtrack
                    if(!a1.hasCompleted){
                        cout << "Will backtrack agent " << a1.id << " old coordinates " << src1.first << " " << src1.second << endl;
                        Pair aux = backtrack(src1, a1.id);
                        if(aux.first != -1 && aux.second != -1){
                            src1 = aux;
                        }
                        
                        cout << "Will backtrack agent " << a1.id << " new coordinates " << src1.first << " " << src1.second << endl;
                    }else if(!a2.hasCompleted){
                        cout << "Will backtrack agent " << a2.id << " old coordinates " << src2.first << " " << src2.second << endl;
                        Pair aux = backtrack(src2, a2.id);
                        if(aux.first != -1 && aux.second != -1){
                            src2 = aux;
                        }
                        cout << "Will backtrack agent " << a2.id << " new coordinates " << src2.first << " " << src2.second << endl;
                    }
                }
                
                // Clear the shared DS
                sharedVector.clear();
                cout << "Clearing grid" << endl;
                printSharedMapVector();
                // Print the new configuration
                printGrid();
            }else{
                cout << "Not all agents could be routed" << endl << endl;
                // Print the final configuration if so
                printGrid();
                break;
            }
        }
    return 0;
}
