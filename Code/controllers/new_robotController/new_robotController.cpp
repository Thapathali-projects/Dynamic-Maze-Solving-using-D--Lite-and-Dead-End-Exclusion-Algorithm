#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Display.hpp>
#include<webots/keyboard.hpp>
#include <iomanip>
#include <iostream>
#include<string>
#include <queue>
#include <cmath>
#include <list>

#define TIME_STEP 64
#define pi 3.141592
#define cellNumber 16
#define X cellNumber-1
#define Y cellNumber-1

int startId = 0;
char startDirection = 'N'; //-1.5707 //3.1415
using namespace webots;
using namespace std;

Display * display;
int uniqueCellsVisited = 0;
int realCellsVisited = 0;
void updateCurrentCellInfo(char currentDirection, bool frontObstacle, bool rightObstacle, bool leftObstacle, int id);
void updateNeighboringCells(int id);
char getTargetDirection(int currentId,int targetId);
int getNextId(char currentDirection, bool frontObstacle, bool rightObstacle, bool leftObstacle, int id);
bool shouldRotateClockwiseFun(char currentDirection, char nextDirection);
bool tryClosingCell(int id);
int displaySize = 1600;
int cellSize = displaySize / (cellNumber + 2);
int mazeSize = cellSize * cellNumber;
int padding = cellSize;

/*#region Display related */
void showNArrow(int x,int y){
  display -> drawText("\u25B2", cellSize * (x + 0.3) + 1.1*padding, mazeSize - cellSize * (y - 0.4));  
}
void showSArrow(int x,int y){
  display -> drawText("\u25BC", cellSize * (x + 0.3) + 1.15*padding, mazeSize - cellSize * (y - 0.45));  
}
void showEArrow(int x,int y){
  display -> drawText("\u25B6", cellSize * (x + 0.3) + 1.2*padding, mazeSize - cellSize * (y - 0.4));  
}
void showWArrow(int x,int y){
  display -> drawText("\u25C0", cellSize * (x + 0.3) + 1.1*padding, mazeSize - cellSize * (y - 0.4));  
}
void showRobotPos(int x, int y,char dir) {
  display -> fillRectangle(cellSize * (x + 0.3) + padding-5, mazeSize - cellSize * (y - 0.3)-5, 0.65 * cellSize, 0.67 * cellSize);
  display -> setColor(0x000000);
  switch(dir){
  case 'N':
  showNArrow(x,y);
  break;
  case 'S':
  showSArrow(x,y);
  break;
  case 'E':
  showEArrow(x,y);
  break;
  default:
  showWArrow(x,y);
  break;
  }
  display -> setColor(0xFFFFFF);
}
void eraseRobotPos(int x, int y) {
  display -> setColor(0x000000);
  display -> fillRectangle(cellSize * (x + 0.3) + padding-5, mazeSize - cellSize * (y - 0.3)-5, 0.65 * cellSize, 0.67 * cellSize);
  display -> setColor(0xFFFFFF);
}


void showCellValue(int x, int y, int data) {
  display -> drawText(to_string(data), cellSize * x + cellSize * (0.55 - 0.1 * to_string(data).length()) + padding, mazeSize - cellSize * y + 0.44 * cellSize);
}

void eraseCellValue(int x, int y, int data) {
  display -> setColor(0x000000);
  display -> drawText(to_string(data), cellSize * x + cellSize * (0.55 - 0.1 * to_string(data).length()) + padding, mazeSize - cellSize * y + 0.44 * cellSize);
  display -> setColor(0xFFFFFF);
}

void showEastWall(int x, int y) {
  display -> fillRectangle(cellSize * (x + 1) + padding, mazeSize - cellSize * y, 0.1 * cellSize, 1.1 * cellSize);
}

void showWestWall(int x, int y) {
  display -> fillRectangle(cellSize * x + padding, mazeSize - cellSize * y, 0.1 * cellSize, 1.1 * cellSize);
}

void showNorthWall(int x, int y) {
  display -> fillRectangle(cellSize * x + padding, mazeSize - cellSize * y, 1.1 * cellSize, 0.1 * cellSize);
}

void showSouthWall(int x, int y) {
  display -> fillRectangle(cellSize * x + padding, mazeSize - cellSize * (y - 1), 1.1 * cellSize, 0.1 * cellSize);
}

void displayClosedCell(int id){
  int x = id % cellNumber, y = id / cellNumber;
  display -> setColor(0xFF0000);
  display -> fillRectangle(cellSize * (x + 0.15) + padding, mazeSize - cellSize * y + 0.15 * cellSize, 0.85 * cellSize, 0.1 * cellSize);
  display -> fillRectangle(cellSize * (x + 0.15) + padding, mazeSize - cellSize * y + 0.15 * cellSize, 0.1 * cellSize, 0.85 * cellSize);
  display -> fillRectangle(cellSize * (x + 0.9) + padding, mazeSize - cellSize * y + 0.15 * cellSize, 0.1 * cellSize, 0.85 * cellSize);
  display -> fillRectangle(cellSize * (x + 0.15) + padding, mazeSize - cellSize * y + 0.9 * cellSize, 0.85 * cellSize, 0.1 * cellSize);
  display -> setColor(0xFFFFFF);
}

void drawWalls(int x, int y, bool northPath, bool southPath, bool eastPath, bool westPath) {
  if (not(northPath)) showNorthWall(x, y);
  if (not(southPath)) showSouthWall(x, y);
  if (not(eastPath)) showEastWall(x, y);
  if (not(westPath)) showWestWall(x, y);
}

void eraseAllCosts(){
  for(int i = 0;i<cellNumber;i++){
    for(int j = 0;j<cellNumber;j++){
      eraseRobotPos(i,j);
    }
  }
}
void showArrow(int id,int goalId);
void printOpenList();
void showRhs();
/*#endregion */

/*#region DStarLite*/
float manhattanDistance(int id) {
  int x = id % cellNumber, y = id /cellNumber;
  int x2 = startId % cellNumber, y2 = startId /cellNumber;
  return pow((x2 - x)* (x2 - x) + (y2 - y) * (y2 - y) , 0.5);
}

void updateVertex(int id, int goalId);
void calculateShortestPath(int goalId);
/*#endregion*/

class Cell {
  public:
  int north = -1, south = -1, east = -1, west = -1; // 1 = there is wall, 0 = there is no wall, -1 = unknown
  int cost = 0; // 0 = cell not explored
  int id;
  int successorId;
  bool obstacle = false;
  list < int > predecessorList;
  float g = INFINITY;
  float rhs = INFINITY;
  float key1() const {return (min(g, rhs) + manhattanDistance(id));}
  float key2() const {return min(g, rhs);}
};
Cell maze[cellNumber*cellNumber];

int getGoal() {
  for (int i = 0; i < cellNumber; i++) {
    for (int j = 0; j < cellNumber; j++) {
      if (maze[i+j*cellNumber].cost < 500 && maze[i+j*cellNumber].east == 0 && maze[i+j*cellNumber].north == 0 && maze[(i+1)+(j+1)*cellNumber].west == 0 && maze[(i+1)+(j+1)*cellNumber].south == 0)
      {
        return (i+j*cellNumber);
      }
    }
  }
  return -1;
}
bool operator < (const Cell & cellA,const Cell & cellB) {
  return !((cellA.key1() < cellB.key1()) || (cellA.key1() == cellB.key1() && cellA.key2() <= cellB.key2()));
}
priority_queue < Cell > openList;


void closeCell(int id) {
  maze[id].north = 1;
  maze[id].south = 1;
  maze[id].east = 1;
  maze[id].west = 1;
  displayClosedCell(id);
}
enum phase {DeadEndExclusion, Waiting, DStarLite, Exception }simulationPhase;
enum movement {Forward, RotateCW, RotateCCW, Stop};
int main() {

  // #region Declaration
  Robot * robot = new Robot();
  DistanceSensor * dsLeft, * dsRight, * dsFront;
  Motor * leftWheel, * rightWheel;
  PositionSensor * leftEncoder, * rightEncoder;
  InertialUnit * imu;
  Keyboard kb;
  // #region Init
  dsLeft = robot -> getDistanceSensor("dsLeft");
  dsRight = robot -> getDistanceSensor("dsRight");
  dsFront = robot -> getDistanceSensor("dsFront");

  leftEncoder = robot -> getPositionSensor("leftEncoder");
  rightEncoder = robot -> getPositionSensor("rightEncoder");
  imu = robot -> getInertialUnit("imu");

  leftWheel = robot -> getMotor("leftWheel");
  rightWheel = robot -> getMotor("rightWheel");
  display = robot -> getDisplay("display");
  display -> setFont("Verdana", 0.015 * mazeSize, true);

  dsLeft -> enable(TIME_STEP);
  dsRight -> enable(TIME_STEP);
  dsFront -> enable(TIME_STEP);
  leftEncoder -> enable(TIME_STEP);
  rightEncoder -> enable(TIME_STEP);
  imu -> enable(TIME_STEP);

  kb.enable(TIME_STEP); //keyboard

  leftWheel -> setPosition(INFINITY);
  rightWheel -> setPosition(INFINITY);
  leftWheel -> setVelocity(0);
  rightWheel -> setVelocity(0);
  
  float encoderLeftDistance, encoderRightDistance;
  
  float leftDSValue, rightDSValue, frontDSValue;
  bool frontObstacle, rightObstacle, leftObstacle;
  float currentDistance = 0, targetDistance = 0;
  
  int prevId = startId,currentId = startId,targetId=startId;
  char currentDirection = startDirection, targetDirection = startDirection;
  float currentAngle, targetAngle = 0;
  float error = 0, pastError = 0, totalError = 0; // for PID
  int goalId = -1;
  
  simulationPhase = DeadEndExclusion;
  movement botMovement = Forward;
  queue<int> movementQueue; 

  // #region Init Walls around the maze
  for (int i = 0; i < cellNumber; i++) {
    for (int j = 0; j < cellNumber; j++) {
      int id = i + cellNumber * j;
      if (i == 0) {
        maze[id].west = 1;
        showWestWall(i, j);
        showCellValue(-1, j, j);
      }
      if (i == (cellNumber - 1)) {
        maze[id].east = 1;
        showEastWall(i, j);
      }
      if (j == 0) {
        maze[id].south = 1;
        showSouthWall(i, j);
        showCellValue(i, -1, i);
      }
      if (j == (cellNumber - 1)) {
        maze[id].north = 1;
        showNorthWall(i, j);
      }
      maze[id].id =id; 
    }
  }
  switch (startDirection){
  case 'N':
    maze[startId].south = 0;  
    break;
  case 'S':
    maze[startId].north = 0;
    break;
  case 'E':
    maze[startId].west = 0;
    break;
  default:
    maze[startId].east = 0;
    break;
  }
  
  bool newCell = true;
  bool update = false;
  while (robot -> step(TIME_STEP) != -1) {
    leftDSValue = dsLeft -> getValue();
    rightDSValue = dsRight -> getValue();
    frontDSValue = dsFront -> getValue();
    encoderLeftDistance = leftEncoder -> getValue() * 0.02; //multiplied by radius of wheel 
    encoderRightDistance = rightEncoder -> getValue() * 0.02;
    currentAngle = imu -> getRollPitchYaw()[2];
    // if(botMovement != Stop)cout<<currentId<<"->"<<targetId<<endl;
    switch(simulationPhase){
      case DeadEndExclusion:
      if(newCell){ // make new decision       
          prevId = currentId;
          currentId = targetId;
          frontObstacle = frontDSValue == 15 ? false : true;
          leftObstacle = leftDSValue == 15 ? false : true;
          rightObstacle = rightDSValue == 15 ? false : true;     
          
          if (maze[currentId].cost == 0){
            uniqueCellsVisited++;
            maze[currentId].cost++;
            updateCurrentCellInfo(currentDirection, frontObstacle, rightObstacle, leftObstacle, currentId);
            tryClosingCell(currentId);
            updateNeighboringCells(currentId);
          }else{
             maze[currentId].cost++;
          }
          realCellsVisited++;
          if(uniqueCellsVisited >= (cellNumber * cellNumber)) {
            simulationPhase = Waiting;
            cout<<"Cells:256"<<endl;
            cout<<"Press s to start next phase"<<endl;
            break;
          }
        targetId = getNextId(currentDirection,frontObstacle,rightObstacle,leftObstacle,currentId);
        movementQueue.push(targetId);
      }
      break;
      
      case Waiting:
      leftWheel -> setVelocity(0);
      rightWheel -> setVelocity(0);
      if(kb.getKey() == 83){
        eraseAllCosts();
        simulationPhase = DStarLite;
        /*#region D* lite init */
        goalId = getGoal();
        if(goalId == -1){
          simulationPhase = Exception;
          botMovement = Stop;
        }
        cout << "Goal Id:" << goalId << endl;
        maze[goalId].rhs = 0;
        openList.push(maze[goalId]);
        /*#endregion*/
  
        // int currentCellId;
        /*#region Find Shortest Path */
        calculateShortestPath(goalId);
        int ri,temp;
        ri = startId;
          do{
            temp = maze[ri].successorId;
            ri = temp;
            showArrow(ri,goalId);
            movementQueue.push(ri);
          }while(ri != goalId); 
        showRhs();
        prevId = currentId;
        targetId = startId;
        currentId = startId;
       }
        break;

      case DStarLite:
        frontObstacle = frontDSValue == 15 ? false : true;
        leftObstacle = leftDSValue == 15 ? false : true;
        rightObstacle = rightDSValue == 15 ? false : true;     
        update = false;    
        if(newCell) {
            prevId = currentId;
            currentId = targetId;
            if(currentId == goalId){
              cout<<"Destination Reached"<<endl;
              botMovement = Stop;
              break;
            }  
            bool N = false,S = false,E = false,W = false;
            switch (currentDirection) {
              case 'N': // if the bot is facing North direction
                N = frontObstacle;
                E = rightObstacle;
                W = leftObstacle;
                break;
              case 'S': // if the bot is facing South direction
                S = frontObstacle;
                E = leftObstacle;
                W = rightObstacle;
                break;
              case 'E': // if the bot is facing East direction
                N = leftObstacle;
                S = rightObstacle;
                E = frontObstacle;
                break;
              default: // if the bot is facing West direction
                N = rightObstacle;
                S = leftObstacle;
                W = frontObstacle;
                break;
              }
            if(N && getTargetDirection(currentId,movementQueue.front()) == 'N'){
              showNorthWall(currentId % cellNumber, currentId /cellNumber);
              maze[currentId].north = 1;
              maze[currentId + cellNumber].south = 1;
              cout<<"Obstacle Found"<<endl;
              update = true;
            }else if(S && getTargetDirection(currentId,movementQueue.front()) == 'S'){
              showSouthWall(currentId % cellNumber, currentId /cellNumber);
              maze[currentId].south = 1;
              maze[currentId - cellNumber].north = 1;
              update = true;
              cout<<"Obstacle Found"<<endl;
            }else if(E && getTargetDirection(currentId,movementQueue.front()) == 'E'){
              showEastWall(currentId % cellNumber, currentId /cellNumber);
              maze[currentId].east = 1;
              maze[currentId + 1].west= 1;
              update = true;
              cout<<"Obstacle Found"<<endl;
            }else if(W && getTargetDirection(currentId,movementQueue.front()) == 'W'){
              showWestWall(currentId % cellNumber, currentId /cellNumber);
              maze[currentId].west = 1;
              maze[currentId - 1].east = 1;
              update = true;
              cout<<"Obstacle Found"<<endl;
            }
            if(update){
            eraseAllCosts();
            startId = currentId;
            prevId = startId;
            maze[movementQueue.front()].rhs = INFINITY;
            openList.push(maze[movementQueue.front()]);
            if(movementQueue.front()%cellNumber < X) updateVertex(movementQueue.front() + 1, goalId);
            if(movementQueue.front()%cellNumber > 0) updateVertex(movementQueue.front() - 1, goalId);
            if(movementQueue.front()/cellNumber < Y) updateVertex(movementQueue.front() + cellNumber, goalId);
            if(movementQueue.front()/cellNumber > 0) updateVertex(movementQueue.front() - cellNumber, goalId);
            
            botMovement = Stop;
            while(!movementQueue.empty()){
              movementQueue.pop();
            }
            priority_queue < Cell > tempQ = openList;
            priority_queue < Cell > newOpenList;
            while (!tempQ.empty()) {
              // cout<<tempQ.top().id<<"  ";  
              newOpenList.push(tempQ.top());
              tempQ.pop();
            }
            cout<<endl;
            openList = newOpenList;
            calculateShortestPath(goalId);
            // printOpenList();
            cout<<endl;
            int ri,temp;
            ri = currentId;
            if(maze[ri].g > 9999){
              cout<<"No possible path"<<endl;
              botMovement = Stop;
              break;
            }
            do{
              temp = maze[ri].successorId;
              ri = temp;
              showArrow(ri,goalId);
              movementQueue.push(ri);
            }while(ri != goalId); 
          showRhs();
          }
      }
      // cout<<"D* Lite"<<endl;
      break;
      
      case Exception:
      cout<<"No goal found!"<<endl;
      botMovement = Stop;
      break;
    }
        
      if(newCell){
      eraseRobotPos(prevId % cellNumber,prevId / cellNumber);
      showRobotPos(currentId % cellNumber,currentId / cellNumber,currentDirection);
      if(simulationPhase != DStarLite)showCellValue(prevId % cellNumber, prevId /cellNumber, maze[prevId].cost);
    
      newCell = false;   
      if(movementQueue.empty()) continue;
      else{
        targetDistance = currentDistance + 0.18 + abs(targetDistance - currentDistance);
        targetId = movementQueue.front();
        movementQueue.pop();
        if(simulationPhase == DeadEndExclusion){
          cout<<"Cells:"<<uniqueCellsVisited<<endl;
        }
        // cout<<currentId<<"->"<<targetId<<endl;
        targetDirection = getTargetDirection(currentId,targetId);
        // cout<<currentDirection<<"->"<<targetDirection<<endl;
        if(currentDirection == targetDirection) botMovement = Forward;
        else{
          botMovement = shouldRotateClockwiseFun(currentDirection, targetDirection) ? RotateCW:RotateCCW;
          currentDirection = targetDirection;
        }
        if(targetDirection == 'N')targetAngle = 0;
        else if(targetDirection == 'S')targetAngle = pi;
        else if(targetDirection == 'E')targetAngle = -pi * 0.5;
        else targetAngle = pi * 0.5;
      }
    }
    switch(botMovement){
      case Forward:
        if(abs(currentDistance - targetDistance) > 0.01){
          currentDistance = (encoderRightDistance + encoderLeftDistance) / 2;
          if (rightDSValue < 15 and leftDSValue < 15) error = leftDSValue - rightDSValue;
          else error = 0;
          leftWheel -> setVelocity(9 - (0.05 * error + 0.0001 * (pastError - error) + totalError * 0.00001));
          rightWheel -> setVelocity(9 + (0.05 * error + 0.0001 * (pastError - error) + totalError * 0.00001));
          pastError = error;
          totalError += error;
        }
        else newCell = true;
        break;
        
      case RotateCW:
          if (abs(currentAngle - targetAngle) > 0.005) {
          double error = abs(abs(currentAngle) - abs(targetAngle));
            leftWheel -> setVelocity(abs(0.1 + 2*error));
            rightWheel -> setVelocity(-abs(0.1 + 2*error));
        } else { // rotation done
          targetDistance = currentDistance + abs(targetDistance - currentDistance);
          botMovement = Forward;
          showRobotPos(currentId%cellNumber,currentId/cellNumber,currentDirection);
        }
      break;
      
      case RotateCCW:
          if (abs(currentAngle - targetAngle) > 0.005) {
          double error = abs(abs(currentAngle) - abs(targetAngle));
            leftWheel -> setVelocity(-abs(0.1 + 2*error));
            rightWheel -> setVelocity(abs(0.1 + 2*error));
        } else { // rotation done
          targetDistance = currentDistance + abs(targetDistance - currentDistance);
          botMovement = Forward;
          showRobotPos(currentId%cellNumber,currentId/cellNumber,currentDirection);
        }
      break;
      
      case Stop:
        leftWheel -> setVelocity(0);
        rightWheel -> setVelocity(0);
        // cout<<"Movement Stopped"<<endl;
      break;
    }  
  }
  delete robot;
  return 0; // EXIT_SUCCESS
}
void updateCurrentCellInfo(char currentDirection, bool frontObstacle, bool rightObstacle, bool leftObstacle, int id) {
  bool N = -1, S = -1, E = -1, W = -1;
  switch (currentDirection) {
  case 'N': // if the bot is facing North direction
    N = frontObstacle;
    S = false; //there is no obstacle
    E = rightObstacle;
    W = leftObstacle;
    break;
  case 'S': // if the bot is facing South direction
    N = false; //there is no obstacle
    S = frontObstacle;
    E = leftObstacle;
    W = rightObstacle;
    break;
  case 'E': // if the bot is facing East direction
    N = leftObstacle;
    S = rightObstacle;
    E = frontObstacle;
    W = false; //there is no obstacle
    break;
  default: // if the bot is facing West direction
    N = rightObstacle;
    S = leftObstacle;
    E = false;
    W = frontObstacle;
    break;
  }
  if (maze[id].north == -1) maze[id].north = N;
  if (maze[id].south == -1) maze[id].south = S;
  if (maze[id].east == -1) maze[id].east = E;
  if (maze[id].west == -1) maze[id].west = W;
  drawWalls(id%16, id/16, !N, !S, !E, !W);
}

bool tryClosingCell(int id) {
  int x = id % 16,y = id /16;
  if (maze[id].cost >= 100)return false; // Already Closed
  bool N = maze[id].north == 1 ? true : false;
  bool S = maze[id].south == 1 ? true : false;
  bool E = maze[id].east == 1 ? true : false;
  bool W = maze[id].west == 1 ? true : false;
  if ((N and S and E) or(N and S and W) or(S and E and W) or(N and E and W)) {
    closeCell(id);
    eraseCellValue(x, y, maze[id].cost);
    if (maze[id].cost == 0) uniqueCellsVisited++;
    maze[id].cost = 500 - uniqueCellsVisited;
    showCellValue(x, y, maze[id].cost);
    if (y < Y) maze[id+cellNumber].south = 1;
    if (y > 0) maze[id-cellNumber].north = 1;
    if (x < X) maze[id+1].west = 1;
    if (x > 0) maze[id-1].east = 1;
    return true;
  } else return false;
}
void updateNeighboringCells(int id) {
  int x,y;
  x = id % cellNumber;
  y = id / cellNumber;
  if (y < Y) {
    if (maze[id + cellNumber].south != 1) maze[id + cellNumber].south = maze[id].north;
    if (tryClosingCell(id + cellNumber)) updateNeighboringCells(id + cellNumber);
  }
  if (y > 0) {
    if (maze[id - cellNumber].north != 1) maze[id - cellNumber].north = maze[id].south;
    if (tryClosingCell(id - cellNumber)) updateNeighboringCells(id - cellNumber);
  }
  if (x < X) {
    if (maze[id + 1].west != 1) maze[id + 1].west = maze[id].east;
    if (tryClosingCell(id + 1)) updateNeighboringCells(id + 1);
  }
  if (x > 0) {
    if (maze[id - 1].east != 1) maze[id - 1].east = maze[id].west;
    if (tryClosingCell(id - 1)) updateNeighboringCells(id - 1 );
  }
}
char getTargetDirection(int currentId,int targetId) {
  int netId = targetId - currentId;//Can yield -1,+1,-16, +16
    if(netId == 16)return 'N';
    else if(netId == -16)return 'S';
    else if(netId == 1)return 'E';
    else return 'W';
}
bool shouldRotateClockwiseFun(char currentDirection, char nextDirection) {
  if (currentDirection == 'N'
    and nextDirection == 'W') return false;
  else if (currentDirection == 'S'
    and nextDirection == 'E') return false;
  else if (currentDirection == 'E'
    and nextDirection == 'N') return false;
  else if (currentDirection == 'W'
    and nextDirection == 'S') return false;
  else return true;
}

int getNextId(char currentDirection, bool frontObstacle, bool rightObstacle, bool leftObstacle, int id) {
  int N = 1000, E = 1000, W = 1000, S = 1000; //don't go out of maze
  int x,y;
  x = id % 16;
  y = id / 16;
  switch (currentDirection) {
  case 'N':
    if (y < Y) N = frontObstacle ? 1000 : maze[id + cellNumber].cost;
    if (y > 0) S = maze[id - cellNumber].cost;
    if (x < X) E = rightObstacle ? 1000 : maze[id + 1].cost;
    if (x > 0) W = leftObstacle ? 1000 : maze[id - 1].cost;

    if (S < N and S < E and S < W) return id-cellNumber;
    else if (W < N and W < E) return id-1;
    else if (E < N) return id+1;
    else return id+cellNumber;
    break;
  case 'S':
    if (y < Y) N = maze[id + cellNumber].cost;
    if (y > 0) S = frontObstacle ? 1000 : maze[id - cellNumber].cost;
    if (x < X) E = leftObstacle ? 1000 : maze[id + 1].cost;
    if (x > 0) W = rightObstacle ? 1000 : maze[id - 1].cost;

    if (N < S and N < W and N < E) return id+cellNumber;
    else if (E < S and E < W) return id+1;
    else if (W < S) return id-1;
    else return id-cellNumber;
    break;
  case 'E':
    if (y < Y) N = leftObstacle ? 1000 : maze[id + cellNumber].cost;
    if (y > 0) S = rightObstacle ? 1000 : maze[id - cellNumber].cost;
    if (x < X) E = frontObstacle ? 1000 : maze[id + 1].cost;
    if (x > 0) W = maze[id - 1].cost;

    if (W < E and W < S and W < N) return id-1;
    else if (N < E and N < S) return id+cellNumber;
    else if (S < E) return id-cellNumber;
    else return id+1;
    break;
  default: // West Direction
    if (y < Y) N = rightObstacle ? 1000 : maze[id + cellNumber].cost;
    if (y > 0) S = leftObstacle ? 1000 : maze[id - cellNumber].cost;
    if (x < X) E = maze[id + 1].cost;
    if (x > 0) W = frontObstacle ? 1000 : maze[id - 1].cost;

    if (E < W and E < N and E < S) return id+1;
    else if (S < W and S < N) return id-cellNumber;
    else if (N < W) return id+cellNumber;
    else return id-1;
    break;
  }
}

void updateVertex(int id, int goalId) {
  if (id != goalId) {
    float minimumValue = INFINITY;
    int x = id % cellNumber, y = id / cellNumber;
    if (x > 0 && maze[id].west == 0) { // Left present
      if (minimumValue > maze[id - 1].g + 1) {
        minimumValue = maze[id- 1].g + 1;
        maze[id].successorId = id - 1;
      }
    }
    if (x < X && maze[id].east == 0) { // Right present
      if (minimumValue > maze[id + 1].g + 1) {
        minimumValue = maze[id + 1].g + 1;
        maze[id].successorId = id + 1;
      }
    }
    if (y > 0 && maze[id].south == 0) { // Bottom present
      if (minimumValue > maze[id - cellNumber].g + 1) {
        minimumValue = maze[id - cellNumber].g + 1;
        maze[id].successorId = id - cellNumber;
      }
    }
    if (y < Y && maze[id].north == 0) { // Top present
      if (minimumValue > maze[id + cellNumber].g + 1) {
        minimumValue = maze[id + cellNumber].g + 1;
        maze[id].successorId = id + cellNumber;
      }
    }
    maze[id].rhs = minimumValue;
    priority_queue < Cell > temp = openList;
    priority_queue < Cell > newOpenList;

    while (!temp.empty()) { // removing id from openlist if it exists
      if (id != temp.top().id) newOpenList.push(temp.top());
      temp.pop();
    }
    if (maze[id].g != maze[id].rhs) newOpenList.push(maze[id]);
    openList = newOpenList;
  }
}
void printOpenList(){
  priority_queue < Cell > temp = openList;
    cout<<"OpenList: ";
    while (!temp.empty()) {
      cout<<temp.top().id<<" ";
      temp.pop();
    }
    cout<<endl;
}

void printRhsAndG(){
  cout<<" "<<endl<<"Rhs:                                                         G:"<<endl;
  for(int j = cellNumber-1;j>=0;j--){
  cout<<setw(2)<<j<<"   ";
    for(int i = 0; i<cellNumber; i++){
      if(maze[i+ j*cellNumber].rhs > 9999)cout<<" ∞ ";
      else cout<<setw(2)<<to_string(int(maze[i+ j*cellNumber].rhs))<< " ";
    }
    cout<<"   "<<setw(2)<<j<<"   ";
    for(int i = 0; i<cellNumber; i++){
      if(maze[i+ j*cellNumber].g > 9999)cout<<" ∞ ";
      else cout<<setw(2)<<to_string(int(maze[i+ j*cellNumber].g))<< " ";
    }
    cout<<endl;
  }
  cout<<" "<<endl<<"    ";
    for(int j = 0;j<1;j++){
      cout<<" ";
      for(int i = 0; i<cellNumber; i++){
        cout<<setw(2)<<i<<" ";
      }
      cout<<"        ";
      for(int i = 0; i<cellNumber; i++){
        cout<<setw(2)<<i<<" ";
      }
  }
  cout<<endl<<endl;
}

void Predecessors(int id,int goalId){
  if ((id % cellNumber) > 0 && maze[id].west == 0 && (id - 1) != goalId) { // Left present
    maze[id].predecessorList.push_back(id - 1);
  }
  if ((id % cellNumber) < X && maze[id].east == 0 && (id + 1) != goalId) { // Right present
    maze[id].predecessorList.push_back(id + 1);
  }
  if ((id/cellNumber) > 0 && maze[id].south == 0 && (id - cellNumber) != goalId) { // Bottom present
    maze[id].predecessorList.push_back(id - cellNumber);
  }
  if ((id/cellNumber) < Y && maze[id].north == 0 && (id + cellNumber)  != goalId) { // Top present
    maze[id].predecessorList.push_back(id + cellNumber);
  }
}

void calculateShortestPath(int goalId){
  int id;
  if(openList.empty())cout<<"Empty prior to:"<<endl;
    while (((openList.top().key1() < maze[startId].key1() || (openList.top().key1() == maze[startId].key1() && openList.top().key2() <= maze[startId].key2())) || maze[startId].rhs != maze[startId].g)) {
      if(openList.empty()){cout<<"No possible path"<<endl;break;}
      // printOpenList();
      id = openList.top().id;
      openList.pop();
      if (maze[id].g > maze[id].rhs) {
        maze[id].g = maze[id].rhs;
        Predecessors(id,goalId);
        for (const int & predecessorId: maze[id].predecessorList)updateVertex(predecessorId, goalId);
      } else {
        maze[id].g = INFINITY;
        for (const int & predecessorId: maze[id].predecessorList)updateVertex(predecessorId, goalId);
        updateVertex(id, goalId);
      }
    }
    // printRhsAndG();
    // printG();
}
void showRhs(){
  // for(int j = cellNumber-1;j>=0;j--){
    // for(int i = 0; i<cellNumber; i++){
      // if(maze[i+ j*cellNumber].rhs > 9999)  display -> drawText("∞", cellSize * i + cellSize * (0.55 - 0.1) + padding, mazeSize - cellSize * j + 0.44 * cellSize);
      // else display -> drawText(to_string(int(maze[i+j*cellNumber].rhs)), cellSize * i + cellSize * (0.55 - 0.1) + padding, mazeSize - cellSize * j + 0.44 * cellSize);
    // }
  // }
}
void showArrow(int id,int goalId){
  int successorId = maze[id].successorId;
  int x = id % cellNumber, y = id/cellNumber;
  if(id != goalId){
    switch (getTargetDirection(id,successorId)){
      case 'N':
      showNArrow(x,y);
      break;
      case 'S':
      showSArrow(x,y);
      break;
      case 'E':
      showEArrow(x,y);
      break;
      default:
      showWArrow(x,y);
      break;
    }
  }
}