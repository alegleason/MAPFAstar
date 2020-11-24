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
 * Helper functions and types file.
 */


#ifndef helper_h
#define helper_h

#define ROW 16
#define COL 18

// Creating a shortcut for int, int pair type
typedef pair<int, int> Pair;

// A structure to hold the neccesary parameters for each cell
struct cell
{
    // Row and Column index of its parent
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

// Creating a shortcut for pair<int, pair<int, int>> type
typedef pair<double, pair<int, int>> pPair;

// This is an strucure which implements the operator overlading (F should be lesser)
struct CompareF {
    bool operator()(cell const& c1, cell const& c2)
    {
        // return "true" if "p1" is ordered  before "p2"
        return c1.f < c2.f;
    }
};

// Function to check whether given cell (row, col) is a valid cell or not
bool isValid(int row, int col)
{
    // Returns true if row number and column number is in range
    return (row >= 0) && (row < ROW) && (col >= 0) && (col < COL);
}

// Function to check whether destination cell has been reached or not
bool isDestination(int row, int col, Pair dest)
{
    if (row == dest.first && col == dest.second){
        return true;
    }
    else{
        return false;
    }
}

// Function to check whether the given cell is blocked or not
bool isUnBlocked(int row, int col, int explorationSpace[ROW][COL])
{
    // Returns true if the cell is not blocked else false
    if (explorationSpace[row][col] == 0){
        return true;
    }else{
        return false;
    }
}

// Function that prints current exploration space
void printGrid(int explorationSpace[ROW][COL]){
    cout << "\nEXPLORATION SPACE " << endl;
    for(int i = 0; i < ROW; i++){
        for(int j = 0; j < COL; j++){
            cout << explorationSpace[i][j] << " ";
        }
        cout << endl;
    }
    cout << endl;
}

// Funnction that prints the content of the shared structure
void printSharedMapVector(vector<pair<pair<int, int>, list<agent>>> sharedVector){
    cout << "Vector content: " << endl;
    for(int i = 0; i < sharedVector.size(); i++){
        cout << "coordinates: " << sharedVector.at(i).first.first << " " <<  sharedVector.at(i).first.second << endl << "agents: " << endl;
        for (auto const& agent:  sharedVector.at(i).second){
            cout << agent.id << " ";
        }
        cout << endl;
    }
}

#endif /* helper_h */
