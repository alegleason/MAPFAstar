// A C++ Program to implement A* Search Algorithm
#include <chrono>
#include <queue>
#include <map>
#include <list>
#include <thread>
#include <mutex>
#include <iostream>
#include <stack>
#include <cfloat>
#include <set>

using namespace std;
using namespace std::chrono;

#define ROW 10
#define COL 10

/* Description of the Grid:
0--> The cell is not blocked
1--> The cell is blocked */
int explorationSpace[ROW][COL] =
{
    {0, 0, 0, 0, 0, 0, 0, 1, 0, 1},
    {0, 0, 1, 0, 1, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 1, 0, 1},
    {0, 1, 0, 0, 1, 0, 0, 1, 1, 0},
    {0, 0, 0, 0, 0, 0, 1, 0, 0, 0},
    {0, 1, 0, 0, 1, 0, 0, 0, 1, 1},
    {0, 0, 0, 0, 0, 0, 0, 1, 1, 0},
    {1, 0, 1, 0, 1, 0, 0, 0, 0, 0},
    {0, 0, 1, 0, 0, 0, 0, 0, 1, 0},
    {0, 1, 0, 0, 0, 0, 0, 1, 0, 0}
};

// Mutex for structure blocking
mutex map_mutex;

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
map<pair<int, int>, list<agent>> sharedMap;

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
    return (row >= 0) && (row < ROW) &&
        (col >= 0) && (col < COL);
}

// A Utility Function to check whether the given cell is
// blocked or not
bool isUnBlocked(int row, int col)
{
    // Returns true if the cell is not blocked else false
    if (explorationSpace[row][col] == 0)
        return (true);
    else
        return (false);
}

// A Utility Function to check whether destination cell has
// been reached or not
bool isDestination(int row, int col, Pair dest)
{
    if (row == dest.first && col == dest.second)
        return (true);
    else
        return (false);
}

// A Utility Function to calculate the 'h' heuristics.
double calculateHValue(int row, int col, Pair dest)
{
    // Return using the Manhattan distance, which returns "total" movements apart
    double h = abs(row - dest.first) + abs(col - dest.second);
    // printf("Manhattan value for cell %i %i is %f\n", row, col, h);
    return h;
}

// Function to update the shared structure
void updateSharedMap(pair<int, int> coordinates, agent currAgent){
    // map_mutex is automatically released when lock goes out of scope
    //cout << "Will add at coordinates: " << coordinates.first << " " << coordinates.second << " agent: " << curr_agent.id << endl;
    const lock_guard<mutex> lock(map_mutex);
    sharedMap[coordinates].push_back(currAgent);
}

// A Utility Function to trace the path from the source
// to destination
void tracePath(cell cellDetails[][COL], Pair dest, agent currAgent)
{
    //printf ("\nThe Path is ");
    int row = dest.first;
    int col = dest.second;

    stack<Pair> Path;

    while (!(cellDetails[row][col].parent_i == row
            && cellDetails[row][col].parent_j == col ))
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
        updateSharedMap(p, currAgent);
        //printf("-> (%d,%d) ",p.first,p.second);
    }

    return;
}

// A Function to find the shortest path between
// a given source cell to a destination cell according
// to A* Search Algorithm
void aStarSearch(Pair src, Pair dest, agent currAgent)
{
    auto start = high_resolution_clock::now();

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
    // This closed list is implemented as a boolean 2D array
    bool closedList[ROW][COL];
    // Fill it with false, since no cell has been included yet
    memset(closedList, false, sizeof (closedList));

    // Declare a 2D array of structure to hold the details
    // of that cell
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
    // 'f' as 0
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
                //printf ("The destination cell is found\n\n");
                auto stop = high_resolution_clock::now();
                auto duration = duration_cast<microseconds>(stop - start);
                //cout << "Total Elapsed Time (Sequential): " << duration.count() << endl;
                tracePath (cellDetails, dest, currAgent);
                foundDest = true;
                return;
            }
            // If the successor is already on the closed
            // list or if it is blocked, then ignore it.
            // Else do the following
            else if (closedList[i-1][j] == false &&
                    isUnBlocked(i-1, j) == true)
            {
                gNew = cellDetails[i][j].g + 1.0; // Adding one to the cost, since I am moving 1 pos
                hNew = calculateHValue (i-1, j, dest); // Manhattan distance call
                fNew = gNew + hNew;

                // If it isn’t on the open list, add it to
                // the open list. Make the current square
                // the parent of this square. Record the
                // f, g, and h costs of the square cell
                //             OR
                // If it is on the open list already, check
                // to see if this path to that square is better,
                // using 'f' cost as the measure.
                if (cellDetails[i-1][j].f == FLT_MAX ||
                        cellDetails[i-1][j].f > fNew)
                {
                    openList.insert( make_pair(fNew, make_pair(i-1, j)));

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

        // Only process this cell if this is a valid one
        if (isValid(i+1, j))
        {
            // If the destination cell is the same as the
            // current successor
            if (isDestination(i+1, j, dest))
            {
                // Set the Parent of the destination cell
                cellDetails[i+1][j].parent_i = i;
                cellDetails[i+1][j].parent_j = j;
                //printf ("The destination cell is found\n\n");
                auto stop = high_resolution_clock::now();
                auto duration = duration_cast<microseconds>(stop - start);
                //cout << "Total Elapsed Time (Sequential): " << duration.count() << endl;
                tracePath (cellDetails, dest, currAgent);
                foundDest = true;
                return;
            }
            // If the successor is already on the closed
            // list or if it is blocked, then ignore it.
            // Else do the following
            else if (closedList[i+1][j] == false &&
                    isUnBlocked(i+1, j) == true)
            {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateHValue(i+1, j, dest);
                fNew = gNew + hNew;

                // If it isn’t on the open list, add it to
                // the open list. Make the current square
                // the parent of this square. Record the
                // f, g, and h costs of the square cell
                //             OR
                // If it is on the open list already, check
                // to see if this path to that square is better,
                // using 'f' cost as the measure.
                if (cellDetails[i+1][j].f == FLT_MAX ||
                        cellDetails[i+1][j].f > fNew)
                {
                    openList.insert(make_pair(fNew, make_pair(i+1, j)));
                    // Update the details of this cell
                    cellDetails[i+1][j].f = fNew;
                    cellDetails[i+1][j].g = gNew;
                    cellDetails[i+1][j].h = hNew;
                    cellDetails[i+1][j].parent_i = i;
                    cellDetails[i+1][j].parent_j = j;
                }
            }
        }

        //----------- 3rd Successor (East) ------------

        // Only process this cell if this is a valid one
        if (isValid (i, j+1))
        {
            // If the destination cell is the same as the
            // current successor
            if (isDestination(i, j+1, dest))
            {
                // Set the Parent of the destination cell
                cellDetails[i][j+1].parent_i = i;
                cellDetails[i][j+1].parent_j = j;
                //printf ("The destination cell is found\n\n");
                auto stop = high_resolution_clock::now();
                auto duration = duration_cast<microseconds>(stop - start);
                //cout << "Total Elapsed Time (Sequential): " << duration.count() << endl;
                tracePath (cellDetails, dest, currAgent);
                foundDest = true;
                return;
            }

            // If the successor is already on the closed
            // list or if it is blocked, then ignore it.
            // Else do the following
            else if (closedList[i][j+1] == false &&
                    isUnBlocked (i, j+1) == true)
            {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateHValue (i, j+1, dest);
                fNew = gNew + hNew;

                // If it isn’t on the open list, add it to
                // the open list. Make the current square
                // the parent of this square. Record the
                // f, g, and h costs of the square cell
                //             OR
                // If it is on the open list already, check
                // to see if this path to that square is better,
                // using 'f' cost as the measure.
                if (cellDetails[i][j+1].f == FLT_MAX ||
                        cellDetails[i][j+1].f > fNew)
                {
                    openList.insert(make_pair(fNew, make_pair(i, j+1)));

                    // Update the details of this cell
                    cellDetails[i][j+1].f = fNew;
                    cellDetails[i][j+1].g = gNew;
                    cellDetails[i][j+1].h = hNew;
                    cellDetails[i][j+1].parent_i = i;
                    cellDetails[i][j+1].parent_j = j;
                }
            }
        }

        //----------- 4th Successor (West) ------------

        // Only process this cell if this is a valid one
        if (isValid(i, j-1))
        {
            // If the destination cell is the same as the
            // current successor
            if (isDestination(i, j-1, dest))
            {
                // Set the Parent of the destination cell
                cellDetails[i][j-1].parent_i = i;
                cellDetails[i][j-1].parent_j = j;
                //printf ("The destination cell is found\n\n");
                auto stop = high_resolution_clock::now();
                auto duration = duration_cast<microseconds>(stop - start);
                //cout << "Total Elapsed Time (Sequential): " << duration.count() << endl;
                tracePath (cellDetails, dest, currAgent);
                foundDest = true;
                return;
            }

            // If the successor is already on the closed
            // list or if it is blocked, then ignore it.
            // Else do the following
            else if (closedList[i][j-1] == false &&
                    isUnBlocked(i, j-1) == true)
            {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateHValue(i, j-1, dest);
                fNew = gNew + hNew;

                // If it isn’t on the open list, add it to
                // the open list. Make the current square
                // the parent of this square. Record the
                // f, g, and h costs of the square cell
                //             OR
                // If it is on the open list already, check
                // to see if this path to that square is better,
                // using 'f' cost as the measure.
                if (cellDetails[i][j-1].f == FLT_MAX ||
                        cellDetails[i][j-1].f > fNew)
                {
                    openList.insert( make_pair (fNew,
                                        make_pair (i, j-1)));

                    // Update the details of this cell
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
    // list is empty, then we conclude that we failed to
    // reach the destiantion cell. This may happen when the
    // there is no way to destination cell (due to blockages)
    if (foundDest == false){
        //printf("Failed to find the Destination Cell\n\n");
        //auto stop = high_resolution_clock::now();
        //auto duration = duration_cast<microseconds>(stop - start);
        //cout << "Total Elapsed Time (Sequential): " << duration.count() << endl;
    }
    
    return;
}

// Cleans the occurences of determined agent on a grid
void removeAgentOccupied(int agentId){
    for(int i = 0; i < ROW; i++){
        for(int j = 0; j < COL; j++){
            if (explorationSpace[i][j] == agentId) {
                explorationSpace[i][j] = 0;
            }
        }
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


void cleanList(int agentToKeepId){
    for(auto elem : sharedMap)
    {
        // If there is more than one collision
        if(elem.second.size() > 1){
            for (auto const& i : elem.second) {
                if(i.priority < highestPriority){
                    highestPriority = i.priority;
                    idHighestPriority = i.id;
                    cout << idHighestPriority << endl;
                }
            }
        }
    }
}


// Driver program to test above function
int main()
{
    int num_threads =  thread::hardware_concurrency();
    
    //thread th1, th2, th3;
    
    cout << "Max. number of threads: " << num_threads << endl;

    // Agent with id 2, priority 2
    agent a1 = {
       2,
       2,
       false
    };
   
    // Agent with id 3, priority 1
    agent a2 = {
       3,
       1,
       false
    };
    
    // Agent with id 3, priority 3
    agent a3 = {
       4,
       3,
       false
    };
    
    // Auxiliary agent for further validations
    agent null_agent = {
        -1,
        -1,
        false
    };
    
    // Source and goal for first agent
    Pair src1 = make_pair(0, 0);

    Pair dest1 = make_pair(6, 4);
    
    // Mark its start and end cells as occupied
    explorationSpace[src1.first][src1.second] = 5;
    explorationSpace[dest1.first][dest1.second] = 5;
    
    // Source and goal for second agent
    Pair src2 = make_pair(5, 5);

    Pair dest2 = make_pair(2, 2);
    
    explorationSpace[src2.first][src2.second] = 6;
    explorationSpace[dest2.first][dest2.second] = 6;
    
    // Source and goal for third agent
    Pair src3 = make_pair(1, 1);

    Pair dest3 = make_pair(4, 4);
    
    explorationSpace[src3.first][src3.second] = 7;
    explorationSpace[dest3.first][dest3.second] = 7;
    
    printGrid();
            
    while (!a1.hasCompleted || !a2.hasCompleted || !a3.hasCompleted) {
        // TODO: Combinations
        if(!a1.hasCompleted){
            thread th1(aStarSearch, src1, dest1, a1);
            th1.join();
        }
        if(!a2.hasCompleted){
            thread th2(aStarSearch, src2, dest2, a2);
            th2.join();
        }
        if(!a3.hasCompleted){
            thread th3(aStarSearch, src3, dest3, a3);
            th3.join();
        }
        // TODO: Move through the map, determine if there are 2 or 3 collisions max. Keep the id with greatest priority and clean map (delete nodes from the linked list that collision).
        int idHighestPriority = 0;
        int highestPriority = 100;
        for(auto elem : sharedMap)
        {
            cout << "Coordinates: " << elem.first.first << " " << elem.first.second << endl;
            // If there is more than one collision
            if(elem.second.size() > 1){
                for (auto const& i : elem.second) {
                    if(i.priority < highestPriority){
                        highestPriority = i.priority;
                        idHighestPriority = i.id;
                        cout << idHighestPriority << endl;
                    }
                }
            }
        }
        
        cout << "Tube with highest priority is: " << idHighestPriority << endl;
        // clean list
        cleanList(idHighestPriority);
        
        bool collisionFound = false, a1Blocked = false, a2Blocked = false, a3Blocked = false;
        if(sharedMap.size() > 0){
            // Check for collisions and update map
            for(auto elem : sharedMap)
            {
                //cout << elem.first.first << " " << elem.first.second << ": " << endl;
                if(elem.second.size() == 1){
                    if(elem.second.front().id == a1.id && !a1Blocked && !a1.hasCompleted){
                        explorationSpace[elem.first.first][elem.first.second] = a1.id;
                    }
                    if(elem.second.front().id == a2.id && !a2Blocked && !a2.hasCompleted){
                        explorationSpace[elem.first.first][elem.first.second] = a2.id;
                    }
                    if(elem.second.front().id == a3.id && !a3Blocked && !a3.hasCompleted){
                        explorationSpace[elem.first.first][elem.first.second] = a3.id;
                    }
                } else if(elem.second.size() > 1) {
                    collisionFound = true;
                    agent first = elem.second.front();
                    cout << "first agent is: " << first.id << endl;
                    elem.second.pop_front();
                    agent second = elem.second.front();
                    cout << "second agent is: " << second.id << endl;
                    elem.second.pop_front();
                    agent third = null_agent;
                    
                    if(elem.second.size() > 0){
                        third = elem.second.front();
                        cout << "third agent is: " << third.id << endl;
                    }
                    
                    if(third.id != -1){
                        cout << "gola" << endl;
                        // Get tube with most priority
                        if (first.priority < second.priority && first.priority < third.priority) {
                            explorationSpace[elem.first.first][elem.first.second] = first.id;
                            if(first.id == a1.id){
                                a2Blocked = true;
                                a3Blocked = true;
                                cout << "a1 has finished!" << endl;
                                a1.hasCompleted = true;
                            }else if(first.id == a2.id){
                                a1Blocked = true;
                                a3Blocked = true;
                                a2.hasCompleted = true;
                                cout << "a2 has finished!" << endl;
                            }else{
                                a1Blocked = true;
                                a2Blocked = true;
                                a3.hasCompleted = true;
                                cout << "a3 has finished!" << endl;
                            }
                            if((second.id == a1.id && !a1.hasCompleted) || (second.id == a2.id && !a2.hasCompleted) || (second.id == a3.id && !a3.hasCompleted)){
                                cout << "removing " << second.id << endl;
                                removeAgentOccupied(second.id);
                            }
                            if((third.id == a1.id && !a1.hasCompleted) || (third.id == a2.id && !a2.hasCompleted) || (third.id == a3.id && !a3.hasCompleted)){
                                cout << "removing " << third.id << endl;
                                removeAgentOccupied(third.id);
                            }
                        }else if(second.priority < first.priority && second.priority < third.priority){
                            explorationSpace[elem.first.first][elem.first.second] = second.id;
                            if(first.id == a1.id){
                                a2Blocked = true;
                                a3Blocked = true;
                                a1.hasCompleted = true;
                                cout << "a1 has finished!" << endl;
                            }else if(first.id == a2.id){
                                a1Blocked = true;
                                a3Blocked = true;
                                a2.hasCompleted = true;
                                cout << "a2 has finished!" << endl;
                            }else{
                                a1Blocked = true;
                                a2Blocked = true;
                                a3.hasCompleted = true;
                                cout << "a3 has finished!" << endl;
                            }
                            if((first.id == a1.id && !a1.hasCompleted) || (first.id == a2.id && !a2.hasCompleted) || (first.id == a3.id && !a3.hasCompleted)){
                                cout << "removing " << first.id << endl;
                                removeAgentOccupied(first.id);
                            }
                            if((third.id == a1.id && !a1.hasCompleted) || (third.id == a2.id && !a2.hasCompleted) || (third.id == a3.id && !a3.hasCompleted)){
                                cout << "removing " << third.id << endl;
                                removeAgentOccupied(third.id);
                            }
                        } else if(third.priority < first.priority && third.priority < second.priority){
                            explorationSpace[elem.first.first][elem.first.second] = third.id;
                            if(first.id == a1.id){
                                a2Blocked = true;
                                a3Blocked = true;
                                a1.hasCompleted = true;
                                cout << "a1 has finished!" << endl;
                            }else if(first.id == a2.id){
                                a1Blocked = true;
                                a3Blocked = true;
                                a2.hasCompleted = true;
                                cout << "a2 has finished!" << endl;
                            }else{
                                a1Blocked = true;
                                a2Blocked = true;
                                a3.hasCompleted = true;
                                cout << "a3 has finished!" << endl;
                            }
                            if((second.id == a1.id && !a1.hasCompleted) || (second.id == a2.id && !a2.hasCompleted) || (second.id == a3.id && !a3.hasCompleted)){
                                cout << "removing " << second.id << endl;
                                removeAgentOccupied(second.id);
                            }
                            if((first.id == a1.id && !a1.hasCompleted) || (first.id == a2.id && !a2.hasCompleted) || (first.id == a3.id && !a3.hasCompleted)){
                                cout << "removing " << first.id << endl;
                                removeAgentOccupied(first.id);
                            }
                        }
                    }else{
                        cout << "gola2" << endl;
                        if(first.priority < second.priority){
                            explorationSpace[elem.first.first][elem.first.second] = first.id;
                            if(first.id == a1.id){
                                a2Blocked = true;
                                a1.hasCompleted = true;
                                // Remove all occurrences of collisioning other
                                
                            }else{
                                a1Blocked = true;
                                a2.hasCompleted = true;
                            }
                            removeAgentOccupied(second.id);
                        }else{
                            explorationSpace[elem.first.first][elem.first.second] = second.id;
                            if(second.id == a1.id){
                                a2Blocked = true;
                                a1.hasCompleted = true;
                            }else{
                                a1Blocked = true;
                                a2.hasCompleted = true;
                            }
                            removeAgentOccupied(first.id);
                        }
                    }
                }
            }
            
            // Mark all agents as arrived!
            if(!collisionFound){
                cout << "All agents been placed!" << endl;
                a1.hasCompleted = true;
                a2.hasCompleted = true;
                a3.hasCompleted = true;
            }else{
                cout << "Some agents have been placed!" << endl;
            }
            
            sharedMap.clear();
            printGrid();
        }else{
            cout << "Not all agents could be routed" << endl << endl;
            printGrid();
            break;
        }
    }
    return 0;
}
