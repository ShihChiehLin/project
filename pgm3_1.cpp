/*
*  CSCE 5210 Spring 2016
*  Program 3
*  Shih-Chieh Lin
*  5/6/2016
*  This program compute the search problem using Q-learning.
*  explore randomly every time
*  I use CSE03.cse.unt.edu machine to test my codes, and they are compiled well.
*/
#include <iostream>
#include <cstdlib>
#include <iomanip>

using namespace std;

const int qSizeX = 9;//row
const int qSizeY = 15;//column
const double discount = 0.9;// discount value gamma
const double learnvalue=0.9;// learning value alpha
const int startX=2,startY=12;//start state
const int goalX=6,goalY=2;// goal state
const int iteration=100;//repeat time
int R[qSizeX][qSizeY] =  
{
{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-1},
{-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-1},
{-1, 0, 0, 0,-1, 0, 0, 0, 0, 0,-1, 0, 0, 0,-1},
{-1,-1,-1,-1,-1, 0, 0, 0, 0, 0,-1,-1,-1,-1,-1},
{-1, 0, 0, 0, 0, 0, 0, 0, 0, 0,-1, 0, 0, 0,-1},
{-1, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-1},
{-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-1},
{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
};//maze, 3 is goal , 0 is white state, and -1 is gray square

const int goalR=R[goalX][goalY];// R value of goal state
double Q[qSizeX][qSizeY][4];// Q value array
int finalX(int X,int Y);// route x value
int finalY(int X,int Y);// route y value
void run(int startX,int startY);// from start run to goal
int action();// random select top/left/right/down
void initialize();// initial all Q value to 0
double maxq(int curX,int curY);//maximum q value
int reward(int X,int Y,int act);
int neX(int X,int Y,int act);// x value after action from (X,Y)
int neY(int X,int Y,int act);// y value after action from (X,Y)
double sample(int curX,int curY,int act);

int main(){
  initialize();
  for(int s = 0; s < iteration;s++){
    run(startX,startY);
  } 
/////show all Q value in the array
  cout<<"iteration:"<<iteration<<", discount: "<<discount<<", learningvalue: "<<learnvalue<<endl;
  cout<<"q-values:"<<endl;
  for(int i = 0; i <= (qSizeX - 1); i++){
    cout<<"Row(X): "<<i<<" "<<"\n";
    for(int j = 0; j <= (qSizeY - 1); j++){
      cout<<"Column(Y): "<<setw(2)<<j<<" "<<"("<<setw(7)<<maxq(i,j)<<") ";
      /*
      for(int k = 0; k <= 3; k++){
        cout<<setw(10)<<Q[i][j][k];
        if(j <= qSizeY - 1){
          if(k<4)
          {cout <<",";}  
        }
      }//k
*/
  //cout<<maxq(i,j)<<" "<<j;
    cout<<" ";
    }//j
cout<<"\n";
  }//i
//

  cout<<"All direction in the maze:"<<endl;
  cout<<"d is down, t is top, l is left, r is right, G is goal, M is more direction"<<endl;
  for(int i=0;i<=qSizeX-1;i++)
  {
    for(int j=0;j<=qSizeY-1;j++)
    {
//    cout<<" "<<maxq(i,j)<<" ";
      if(i==goalX && j==goalY){cout<<" G ";}
      else{
        if(maxq(i,j)==Q[i][j][0]){cout<<" t ";}// top
        else if(maxq(i,j)==Q[i][j][1]){cout<<" l ";}//left
        else if(maxq(i,j)==Q[i][j][2]){cout<<" r ";}//right
        else if(maxq(i,j)==Q[i][j][3]){cout<<" d ";}//down
        else{cout<<" M ";}//more than 1 direction are the same
      }
    }
    cout<<endl; 
  }
  cout <<endl;
  cout<<"Route:"<<endl;
  int wayX=startX, wayY=startY;//start point
  while(wayX!=goalX ||  wayY!=goalY)
  {
    int newX1=finalX(wayX,wayY);
    int newY1=finalY(wayX,wayY);
    cout<<"("<<newX1<<" , "<<newY1<<")";
    wayX=newX1;
    wayY=newY1;
  }
  cout<<endl;
  return 0;
}

int finalX(int x,int y){//route x value
  int curX=x,curY=y;
  int maxX,maxY;
  int X,Y;
  double max=0.0;
  if(Q[curX][curY][0]>=max && curX!=0)
  { 
    max=Q[curX][curY][0];
    X=neX(curX,curY,0);
  }
  else
  {max=max;}

  if(max<=Q[curX][curY][3] && curX!=qSizeX-1)
  { 
    max=Q[curX][curY][3];
    X=neX(curX,curY,3);
  }
  else //if(max>Q[curX+1][curY])
  {max=max;}

  if(Q[curX][curY][2]>=max && curY!=qSizeY-1)
  { 
    max=Q[curX][curY][2];
    X=neX(curX,curY,2);
  }
  else //if(Q[curX][curY+1]<max)
  {max=max;}

  if(Q[curX][curY][1]>=max && curY!=0)
  { 
    max=Q[curX][curY][1];
    X=neX(curX,curY,1);
  }
  else //if(Q[curX][curY-1]<max)
  {max=max;}
  return X;
}

int finalY(int x,int y){// route y value
  int curX=x,curY=y;
  int maxX,maxY;
  int X,Y;
  double max=0.0;
  if(Q[curX][curY][0]>=max && curX!=0)
  {
    max=Q[curX][curY][0];
    Y=neY(curX,curY,0);
  }
  else
  {max=max;}

  if(max<=Q[curX][curY][3] && curX!=qSizeX-1)
  { 
    max=Q[curX][curY][3];
    Y=neY(curX,curY,3);
  }
  else //if(max>Q[curX+1][curY])
  {max=max;}

  if(Q[curX][curY][2]>=max && curY!=qSizeY-1)
  {
    max=Q[curX][curY][2];
    Y=neY(curX,curY,2);
  }
  else //if(Q[curX][curY+1]<max)
  {max=max;}

  if(Q[curX][curY][1]>=max && curY!=0)
  {
    max=Q[curX][curY][1];
    Y=neY(curX,curY,1);
  }
  else //if(Q[curX][curY-1]<max)
  {max=max;}
  return Y;
}

void run(int startX,int startY){//run from start state to goal state in the maze
  int  curX=startX;
  int  curY=startY;
  do{
    int act= action();
    Q[curX][curY][act] = ((1-learnvalue)*Q[curX][curY][act])+(learnvalue*sample(curX,curY,act));// Q value compute
    curX=neX(curX,curY,act);// new X
    curY=neY(curX,curY,act);// new Y
  //cout<<"("<<curX<<" , "<<curY<<")";
  }while(curX!=goalX || curY!=goalY);
  if(curX==goalX && curY==goalY)
  {
    Q[curX][curY][0]==learnvalue*R[curX][curY];
    Q[curX][curY][1]==learnvalue*R[curX][curY];
    Q[curX][curY][2]==learnvalue*R[curX][curY];
    Q[curX][curY][3]==learnvalue*R[curX][curY];
  }
}

int action()//move randomly every time
{
  int pact;
  pact=rand()%4;//0 is top, 1 is left, 2 is right, 3 is down
  return pact;
}

void initialize(){// set all Q value to 0
  srand((unsigned)time(0));
  for(int i = 0; i <= (qSizeX - 1); i++){
    for(int j = 0; j <= (qSizeY - 1); j++){
      for(int k = 0; k <=  3; k++){
        Q[i][j][k] = 0;
      }
    }
  }
}

double maxq(int curX,int curY){//maximum value of Q value of state(curX,curY)
  double max=0.0;
  if(Q[curX][curY][0]>=max)
  {max=Q[curX][curY][0];}
  else
  {max=max;}

  if(max<=Q[curX][curY][1])
  {max=Q[curX][curY][1];}
  else //if(max>Q[curX+1][curY])
  {max=max;}

  if(Q[curX][curY][2]>=max)
  {max=Q[curX][curY][2];}
  else //if(Q[curX][curY+1]<max)
  {max=max;}

  if(Q[curX][curY][3]>=max)
  {max=Q[curX][curY][3];}
  else //if(Q[curX][curY-1]<max)
  {max=max;}

  return max;
}

int reward(int X,int Y,int act){//Reward of every state
  if( act==0 ){
    if(X!=0) {
      X=X-1;
      Y=Y;}
    else{
     X=X;
     Y=Y;}
  }//up
  if( act==1 ){
    if( Y!=0){
      Y=Y-1;
      X=X;}
    else{
     X=X;
     Y=Y;}
  }//left
  if( act==2){
    if( Y!=qSizeY-1){
      Y=Y+1;
      X=X;}
    else
    {X=X;
     Y=Y;}
  }//right
  if( act==3 ){
    if( X!=qSizeX-1){
      X=X+1;
      Y=Y;}
    else
    {X=X;
     Y=Y;}
  }//down
  return R[X][Y];
}

int neX(int X,int Y,int act){// x value after act from (X,Y)
  if( act==0 ){
    if(X!=0) {
      X=X-1;
      Y=Y;}
    else
    {X=X;
     Y=Y;}
  }//up
  if( act==1 ){
    if( Y!=0){ 
      Y=Y-1;
      X=X;}
    else
    {X=X;
     Y=Y;}
  }//left
  if( act==2){
    if( Y!=qSizeY-1){ 
      Y=Y+1;
      X=X;}
    else
    {X=X;
     Y=Y;}
  }//right
  if( act==3 ){
    if( X!=qSizeX-1){
      X=X+1;
      Y=Y;}
    else
    {X=X;
     Y=Y;}
  }//down
  return X;
}

int neY(int X,int Y,int act){// y value after act from (X,Y)
  if( act==0 ){
    if(X!=0) {
      X=X-1;
      Y=Y;}
    else
    {X=X;
     Y=Y;}
  }//up
  if( act==1 ){
    if( Y!=0){
      Y=Y-1;
      X=X;}
    else
    {X=X;
     Y=Y;}
  }//left
  if( act==2){
    if( Y!=qSizeY-1){
      Y=Y+1;
      X=X;}
    else
    {X=X;
     Y=Y;}
  }//right
  if( act==3 ){
    if( X!=qSizeX-1){
      X=X+1;
      Y=Y;}
    else
    {X=X;
     Y=Y;}
  }//down
  return Y;
}

double sample(int curX,int curY,int act){//sample value
  int newX=neX(curX,curY,act);
  int newY=neY(curX,curY,act);
  int reward1;
  if( newX==goalX && newY==goalY )
  {reward1=3;} 
  else
  {
    reward1=reward(curX,curY,act);
  }
  double maxsample=discount*maxq(newX,newY);
  return (double)reward1+maxsample;
  //return (reward(curX,curY,act)+(discount*maxq(newX,newY)));
}
