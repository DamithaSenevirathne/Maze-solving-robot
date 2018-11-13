
/*#############################################################################################################################################################################*/

/***********************************************************MAZE SOLVING ROBOT CODE WRITTEN BY TEAM FURY************************************************************************/

/**********************************************************Faculty of Engineering University of Moratuwa************************************************************************/

/*                              Members - G.D. Seneviratne, D.B. Senevirathne, R. Ranganayake, K.M.B.G.B. Gunarathne, D.Madushan                                               */


/*#############################################################################################################################################################################*/

/*
  
  pre defined 14 states of the robot
  lines represent a wall
        _  
  | 1   2   3|    _ 4
   _    _
  |5    6|  _| 7  |_ 8
        _
  |9|   _10 
   __   __              _
  |11|  __| 12 |__| 13 |_ 14  
  

*/

/*included QueueArrat Library*/
#include <QueueArray.h>

/*Size of the maze pre defined*/
#define R 14
#define C 14

/*****************************/

/*************************************************************** EXAMPLE MAZE *****************************************************************************/

byte premat[R][C] = {
  

  {6,10,10,2,10,10,5,3,10,2,6,10,5,11,},
  {9,6,5,3,5,11,7,1,11,7,4,14,9,9,},
  {9,13,9,13,7,8,6,8,9,6,10,5,7,1,},
  {7,2,8,6,2,10,8,12,1,7,5,7,10,8,},
  {6,8,12,4,7,5,6,2,8,11,9,6,10,14,},
  {7,10,5,12,5,9,9,7,10,1,7,0,10,5,},
  {11,6,8,6,1,7,8,6,5,9,6,8,11,9,},
  {9,7,5,9,13,6,5,9,7,8,9,6,8,9,},
  {9,6,8,7,10,8,7,1,11,6,8,7,2,8,},
  {3,8,6,10,10,10,5,9,9,9,11,6,8,11,},
  {3,14,9,12,10,5,9,7,8,9,9,9,11,9,},
  {7,5,7,5,6,8,7,10,10,8,9,7,4,1,},
  {11,3,5,9,7,10,10,5,6,10,4,10,5,9,},
  {7,8,13,7,10,10,14,7,4,10,10,14,7,8,}   
  
  };

/************************************************************************************************************************************************************/


QueueArray <byte> rq;
QueueArray <byte> cq;
//QueueArray <byte> path;


/*------------------------------------------------------------------time--------------------------------------------------------------------------------------------------*/

double start = millis();


/*------------------------------------------------------------------inputs--------------------------------------------------------------------------------------------------*/

byte sr = 0;
byte sc = 0;


byte goalX = 7;
byte goalY = 6;

byte r,c;
int rr,cc;

byte move_count = 0;
bool reached_end = false;
byte nodes_left_in_layer =1;
byte nodes_in_next_layer =0;


byte TopLeftArr[3] = {5,11,14};
byte TopRightArr[3] = {6,12,11};
byte BottomLeftArr[3] = {8,13,14};
byte BottomRightArr[3] = {7,13,12};

byte TopLeftVal = premat[0][0];
byte TopRightVal = premat[0][R-1];
byte BottomLeftVal = premat[R-1][0];
byte BottomRightVal = premat[R-1][R-1];

bool Flipped = false;

byte mat[R][C] = {
  

  {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0}      
  
  };


/*------------------------------------------------------------------inputs--------------------------------------------------------------------------------------------------*/

byte visited[R][C] = {
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0}

  
  };


byte prev[R][C] = {

  {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0}

};

/*-------------------------------------flipping par--------------------------------------------------------------------------------------------------*/



void flipSSArray (){ 
  for (int i=0; i< R; i++)
  {
    for (int j=0; j<R; j++){
        mat[i][j] = premat[i][R-1-j];
      } 
  }
  sr = 0;
  sc = 13;
  TopLeftVal = mat[0][0];
  TopRightVal = mat[0][R-1];
  BottomLeftVal = mat[R-1][0];
  BottomRightVal = mat[R-1][R-1];
}


void flipUDArray (){ 
  for (int i=0; i< R; i++)
  {
    for (int j=0; j<R; j++){
        mat[R-1-i][j] = premat[i][j];
      } 
  }
  sr = 13;
  sc = 0;
  TopLeftVal = mat[0][0];
  TopRightVal = mat[0][R-1];
  BottomLeftVal = mat[R-1][0];
  BottomRightVal = mat[R-1][R-1];
}

void flipTwiceArray (){ 
  for (int i=0; i< R; i++)
  {
    for (int j=0; j<R; j++){
        mat[R-1-i][j] = premat[i][R-1-j];
      } 
  }
  sr = 13;
  sc = 13;
  TopLeftVal = mat[0][0];
  TopRightVal = mat[0][R-1];
  BottomLeftVal = mat[R-1][0];
  BottomRightVal = mat[R-1][R-1];
}

void noFlipArray (){ 
  for (int i=0; i< R; i++)
  {
    for (int j=0; j<R; j++){
        mat[i][j] = premat[i][j];
      } 
  }
  sr = 0;
  sc = 0;
  TopLeftVal = mat[0][0];
  TopRightVal = mat[0][R-1];
  BottomLeftVal = mat[R-1][0];
  BottomRightVal = mat[R-1][R-1];
}


bool isIn(byte arrayList[3] , byte val){

  if (val==arrayList[0] || val == arrayList[1] || val==arrayList[2]){
    return true;    
    }
    else{
      return false;
      }  
}

void flipSuccessCheck(){
  
  if (isIn(TopRightArr,TopRightVal) && isIn(BottomLeftArr,BottomLeftVal) && isIn(BottomRightArr,BottomRightVal)){
    Flipped=true;
    //call maze solver method       //

    bfs(sr,sc,goalX,goalY);

    
  }
  
}
void flipArray(){

  noFlipArray();
  flipSuccessCheck();
  byte num = 0; 
  
    while (Flipped==false){

      if (num == 0 ){
        flipSSArray();
        flipSuccessCheck();
        }
       if (num == 1 ){
        flipUDArray();
        flipSuccessCheck();
        }
       if (num == 2 ){
        flipTwiceArray();
        flipSuccessCheck();
        }
       num++;            
      
      }
}


/*-------------------------------------flipping part --------------------------------------------------------------------------------------------------*/
/**************************************************flipping part adjust the robot direction when started************************************************/


bool check(int x, int y){

    bool isTrue = true;

    if (x<0 || y<0){
      
      isTrue = false;
      }

    if (rr>=R || cc>=C){
      
      isTrue = false;
      }

    if (visited[x][y] == 1){
      
      isTrue = false;
      }

    if (mat[x][y]==0){
      isTrue = false;
      }
  
  
    return isTrue;
  
  }

void arrayRefresh(byte arrayList[R][C]){
  
  for (int i=0; i<R;i++){
    
    
    for (int j=0; j<C;j++){
      
      arrayList[i][j] = 0;
      
      
      }
    
    
    }
  
  
  
  }

void codeFunc(byte rr,byte cc){
  
  rq.push(rr);
  cq.push(cc);
  visited[rr][cc] = 1;
  nodes_in_next_layer++;  
  
  }

byte genearatePoint(byte x,byte y){

  return R*x + y;
  
  
  }

byte generateX(byte val){
  
  return val/R;
  
  
  }

byte generateY(byte val){
  
  return val%R;
  
  
  }

void exploreNeighbours(byte state){


    /*------------------state 0--------------------------*/

  

  if (state == 0){
    
    rr = r + 1;
    cc = c;
    if (check(rr,cc) == true){      
      codeFunc(rr,cc);
      prev[rr][cc] = genearatePoint(r,c);    
    }

    rr = r-1;
    cc = c;
    if (check(rr,cc) == true){      
      codeFunc(rr,cc);
      prev[rr][cc] = genearatePoint(r,c);    
    }

    rr = r;
    cc = c+1;
    if (check(rr,cc) == true){      
      codeFunc(rr,cc);
      prev[rr][cc] = genearatePoint(r,c);    
    }

    rr = r;
    cc = c-1;
    if (check(rr,cc) == true){      
      codeFunc(rr,cc);
      prev[rr][cc] = genearatePoint(r,c);
    }
    
    
    }



  

  /*------------------state 1--------------------------*/

  

  if (state == 1){
    
    rr = r + 1;
    cc = c;
    if (check(rr,cc) == true){      
      codeFunc(rr,cc);
      prev[rr][cc] = genearatePoint(r,c);   
    }

    rr = r;
    cc = c+1;
    if (check(rr,cc) == true){      
      codeFunc(rr,cc);
      prev[rr][cc] = genearatePoint(r,c);
    }

    rr = r - 1;
    cc = c;
    if (check(rr,cc) == true){      
      codeFunc(rr,cc);
      prev[rr][cc] = genearatePoint(r,c);    
    }
    
    
    }


    /*------------------state 2--------------------------*/

    
    if (state == 2){
    
    rr = r;
    cc = c+1;
    if (check(rr,cc) == true){      
      codeFunc(rr,cc);
      prev[rr][cc] = genearatePoint(r,c);
    }

    rr = r;
    cc = c-1;
    if (check(rr,cc) == true){      
      codeFunc(rr,cc);
      prev[rr][cc] = genearatePoint(r,c);  
    }

    rr = r + 1;
    cc = c;
    if (check(rr,cc) == true){      
      codeFunc(rr,cc);
      prev[rr][cc] = genearatePoint(r,c);
    }
    
    
    }

    /*------------------state 3--------------------------*/
    if (state == 3){
    
    rr = r;
    cc = c-1;
    if (check(rr,cc) == true){      
      codeFunc(rr,cc);
      prev[rr][cc] = genearatePoint(r,c);   
    }

    rr = r+1;
    cc = c;
    if (check(rr,cc) == true){      
      codeFunc(rr,cc);
      prev[rr][cc] = genearatePoint(r,c); 
    }

    rr = r - 1;
    cc = c;
    if (check(rr,cc) == true){      
      codeFunc(rr,cc);
      prev[rr][cc] = genearatePoint(r,c);  
    }
    
    
    }

    /*------------------state 4--------------------------*/


    if (state == 4){
    
    rr = r;
    cc = c-1;
    if (check(rr,cc) == true){      
      codeFunc(rr,cc);
      prev[rr][cc] = genearatePoint(r,c);   
    }

    rr = r;
    cc = c+1;
    if (check(rr,cc) == true){      
      codeFunc(rr,cc);
      prev[rr][cc] = genearatePoint(r,c);  
    }

    rr = r - 1;
    cc = c;
    if (check(rr,cc) == true){      
      codeFunc(rr,cc);
      prev[rr][cc] = genearatePoint(r,c);
    }
    
    
    }


    /*------------------state 5--------------------------*/


    if (state == 5){
    
    rr = r+1;
    cc = c;
    if (check(rr,cc) == true){      
      codeFunc(rr,cc);
      prev[rr][cc] = genearatePoint(r,c);
    }

    rr = r;
    cc = c+1;
    if (check(rr,cc) == true){      
      codeFunc(rr,cc);
      prev[rr][cc] = genearatePoint(r,c);
    } 
    
    }

    /*------------------state 6--------------------------*/


    if (state == 6){
    
    rr = r;
    cc = c-1;
    if (check(rr,cc) == true){      
      codeFunc(rr,cc);
      prev[rr][cc] = genearatePoint(r,c);  
    }

    rr = r+1;
    cc = c;
    if (check(rr,cc) == true){      
      codeFunc(rr,cc);
      prev[rr][cc] = genearatePoint(r,c);
    } 
    
    }

    /*------------------state 7--------------------------*/


    if (state == 7){
    
    rr = r-1;
    cc = c;
    if (check(rr,cc) == true){      
      codeFunc(rr,cc);
      prev[rr][cc] = genearatePoint(r,c);    
    }

    rr = r;
    cc = c-1;
    if (check(rr,cc) == true){      
      codeFunc(rr,cc);
      prev[rr][cc] = genearatePoint(r,c);
    } 
    
    }

    
    /*------------------state 8--------------------------*/


    if (state == 8){
    
    rr = r-1;
    cc = c;
    if (check(rr,cc) == true){      
      codeFunc(rr,cc);
      prev[rr][cc] = genearatePoint(r,c); 
    }

    rr = r;
    cc = c+1;
    if (check(rr,cc) == true){      
      codeFunc(rr,cc);
      prev[rr][cc] = genearatePoint(r,c);
    } 
    
    }

    /*------------------state 9--------------------------*/

    if (state == 9){
    
    rr = r-1;
    cc = c;
    if (check(rr,cc) == true){      
      codeFunc(rr,cc);
      prev[rr][cc] = genearatePoint(r,c);  
    }

    rr = r+1;
    cc = c;
    if (check(rr,cc) == true){      
      codeFunc(rr,cc); 
      prev[rr][cc] = genearatePoint(r,c);  
    } 
    
    }


     /*------------------state 10--------------------------*/

    if (state == 10){
    
    rr = r;
    cc = c+1;
    if (check(rr,cc) == true){      
      codeFunc(rr,cc);
      prev[rr][cc] = genearatePoint(r,c);
    }

    rr = r;
    cc = c-1;
    if (check(rr,cc) == true){      
      codeFunc(rr,cc);
      prev[rr][cc] = genearatePoint(r,c);  
    } 
    
    }

    /*------------------state 11--------------------------*/

    if (state == 11){
    
    rr = r+1;
    cc = c;
    if (check(rr,cc) == true){      
      codeFunc(rr,cc);
      prev[rr][cc] = genearatePoint(r,c);   
    } 
    
    }

    /*------------------state 12--------------------------*/

    if (state == 12){
    
    rr = r;
    cc = c-1;
    if (check(rr,cc) == true){      
      codeFunc(rr,cc);
      prev[rr][cc] = genearatePoint(r,c);
    } 
    
    }

    /*------------------state 13--------------------------*/

    if (state == 13){
    
    rr = r-1;
    cc = c;
    if (check(rr,cc) == true){      
      codeFunc(rr,cc);
      prev[rr][cc] = genearatePoint(r,c); 
    } 
    
    }

    /*------------------state 14--------------------------*/

    if (state == 14){
    
    rr = r;
    cc = c+1;
    if (check(rr,cc) == true){      
      codeFunc(rr,cc);
      prev[rr][cc] = genearatePoint(r,c);   
    } 
    
    }


}



/*********************************************  bfs implementation ************************************************************************************************************/

byte bfs(byte sr,byte sc,byte goal_x,byte goal_y){
  
  rq.push(sr);
  cq.push(sc);

  visited[sr][sc] = 1;

  while (rq.count()>0){

    r = rq.pop();
    c = cq.pop();


    if ( r == goalX && c == goalY){
      
      reached_end = true;
      break;
      
      }

    exploreNeighbours(mat[r][c]);

    //Serial.println(nodes_left_in_layer);
    nodes_left_in_layer--;
    if (nodes_left_in_layer ==0){
      
        nodes_left_in_layer = nodes_in_next_layer;

        nodes_in_next_layer = 0;

        
        //path.push(r);
        //path.push(c);
        move_count++;
      
      }


    }

   if (reached_end){
      
    return move_count;
      
      
    }
  
  
  return -1;
  
  }

/*********************************************************************************************************************************************************************/



void setup() {
  // put your setup code here, to run once:

  flipArray();

  
  
  Serial.begin(9600);

  for (int i=0;i<R;i++){
    
    for (int j=0;j<C;j++){
      
        Serial.print(generateX(prev[i][j]));
        Serial.print(" ");
        Serial.print(generateY(prev[i][j]));
        Serial.print("    ");
        
      
      }

    Serial.println(" ");
    
  }

  


/*********************************** *****************backtraking alogorithm***************************************************************************************************/
    
    byte backpoint = genearatePoint(goalX,goalY);

    while (true){

      byte backward = prev[generateX(backpoint)][generateY(backpoint)];

      backpoint = backward;

      

      Serial.print(generateX(backpoint));
      Serial.print(" ");
      Serial.print(generateY(backpoint));
      Serial.println(" ");

      
      

      if (generateX(backpoint) == 0 && generateY(backpoint)==0 ){        
        
        break;
        
        
        }    
      
      }


     double end_ = millis();
     double delta = end_ - start;
     Serial.println(delta);
}

void loop() {
  
}
