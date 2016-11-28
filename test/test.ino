#include <Event.h>
#include <Timer.h>
#include <StackArray.h>

const int Moto1 = 12;
const int Moto2 = 13;
const int Moto3 = 10;
const int Moto4 = 11;
const int echo_1 = 6;
const int trig_1 = 9;
const int echo_2 = 5;
const int trig_2 = 8;
const int echo_3 = 4;
const int trig_3 = 7;
Timer timer;
typedef struct RFL_dis{
  float right;
  float front;
  float left;
} s_dis;
/*--------------可以使用的數值------------------*/
const int f_v = 80;                 // 前進速度
const int v_max = 85;              // 最大速度
# define RIGHT 1
# define FRONT 2
# define LEFT 3
# define STATE_FRONT_RIGHT_LEFT 1
# define STATE_FRONT 1
# define STATE_RIGHT 2
# define STATE_LEFT 3
# define STATE_FRONT_RIGHT 4
# define STATE_FRONT_LEFT 5
# define STATE_RIGHT_LEFT 6
# define STATE_DEAD 7
/*----------------車子現況資料-----------------*/
int right_wheel_v = 0;              // 右輪速度
int left_wheel_v = 0;               // 左輪速度
float distance[4] = {0, 0, 0, 0};   // 三顆sensor的距離
bool no_barrier[4] = {1, 1, 1, 1};  // 有障礙與否
int next_state = 0;                 // 下一個的狀態
int now_state = 0;                  // 現在的狀態
StackArray <s_dis> distance_data;   // 暫存15筆偵測資料
bool state_change_flag = false;     // 當此flag立起，call next_step
int go_forward_id = -1;
int start_time = 0;
int end_time = 0;
/*---------------function 宣告-----------------*/
void debug();
void car_loop();
void writeToSerial();               // debug
void next_step();                   // 根據next_state，讓自走車執行下一步動作
void step_front();                  // case STATE_FRONT's step
void step_right();
void step_left();
void step_front_right();
void step_front_left();
void step_right_left();
void step_dead();
void go_turn(float degree);         // 旋轉度數
void go_forward();                  // 前進 with v=f_v
void go_forward(int v);             // 前進
void go_stop();                     // 停止
void go_left_moto(int v);           // 左輪前進
void go_right_moto(int v);          // 右輪前進
void update_dis();                  // 更新距離
void update_state();                // 更新狀態
void update_status();               // 更新車子現況資料
float dis(int sensor);              // 回傳sensor測到的距離
float detect_angle();               // 偵測車子角度


void setup(){
  pinMode(Moto1, OUTPUT);
  pinMode(Moto2, OUTPUT); 
  pinMode(Moto3, OUTPUT);
  pinMode(Moto4, OUTPUT);
  Serial.begin(9600);
  pinMode (trig_1, OUTPUT);
  pinMode (echo_1, INPUT);
  pinMode (trig_2, OUTPUT);  
  pinMode (echo_2, INPUT);
  pinMode (trig_3, OUTPUT);
  pinMode (echo_3, INPUT);
  // 初始化
  // now_state = STATE_FRONT;
  timer.every(1,car_loop);
  timer.every(1,writeToSerial);
  // timer.every(1000,debug);
}

void debug(){
 go_turn(90);
}

void loop(){
  timer.update();
}

void car_loop(){
  update_dis();     // 更新目前距離
  update_status();  // 更新目前狀態
  if (state_change_flag) next_step(); // 狀態改變的時候，執行下一步
  // next_step();
}

void writeToSerial(){
  // Serial.print("distance: ");
  // for(int i = 1;i<4;i++){
  //   Serial.print(distance[i]);
  //   Serial.print(" ");
  // }
  // Serial.print("    ");
  // Serial.print("block_count: ");
  // Serial.print(block_count);
  // Serial.print("  ");
  // Serial.print("oneblock_last_fdis: ");
  // Serial.print(oneblock_last_fdis);
  // Serial.print("  ");
  // Serial.print("no_barrier: ");
  // for(int i = 1;i<4;i++){
  //   Serial.print(no_barrier[i]);
  //   Serial.print(" ");
  // }
  // Serial.print("distance_data_count: ");
  // Serial.print(distance_data.count());
  // Serial.print(" start_time: ");
  // Serial.print(start_time+200);
  // Serial.print(" now_time: ");
  // Serial.print(millis());
  Serial.print(" state_change_flag:");
  Serial.print(state_change_flag);
  Serial.print(" state: ");
  Serial.print(next_state);
  Serial.println("");
}

void next_step(){
  Serial.print("[next_step]\n");

  switch(now_state){
    case STATE_FRONT:
      step_front();
      break;
    case STATE_RIGHT:
      step_right();
      break;
    case STATE_LEFT:
      step_left();
      break;
    case STATE_FRONT_RIGHT:
      step_front_right();
      break;
    case STATE_FRONT_LEFT:
      step_front_left();
      break;
    case STATE_RIGHT_LEFT:
      step_right_left();
      break;
    case STATE_DEAD:
      step_dead();
      break;
    default:
      go_forward(0);
  }
}

void step_front(){
  Serial.print("[step_front]\n");

  start_time = millis();
  if(go_forward_id == -1){
    Serial.print("[go_forward_id]\n");
    go_forward_id = timer.every(1,go_forward);  
  }
}

void step_right(){
  int acc = 12;
  /* 前進到剩下acc */
  while(dis(FRONT) > acc){
    go_forward(f_v);
  }
  go_stop();
  /*旋轉順時鐘九十度*/
  go_turn(-90);
  /*前進30cm*/
  go_forward(f_v);
  delay(650);
  go_stop();
}

void step_left(){
  int acc = 12;
  /* 前進到剩下acc */
  while(dis(FRONT) > acc){
    go_forward(f_v);
  }
  go_stop();
  /*旋轉逆時鐘九十度*/
  go_turn(90);
  /*前進30cm*/
  go_forward(f_v);
  delay(650);
  go_stop();
}

void step_front_right(){
  /*先右轉*/
}

void step_front_left(){
  /*先左轉*/
}

void step_right_left(){

}

void step_dead(){
  /*倒退回剛剛的岔路*/
  /*如果上一個岔路是左轉則後退右轉; 是右轉則後退左轉90度*/
  /*直線前進30cm*/
}

void go_turn(float degree){
  if(degree > 0){
    /*逆時針*/
    go_left_moto(-80);
    go_right_moto(80);
    delay((float)398/90*degree);
    go_stop();
  }else if(degree < 0){
    /*順時針*/
    go_left_moto(80);
    go_right_moto(-80);
    delay((float)390/90*(-degree));
    go_stop();
  }
}

void go_forward(){
  int acc = 3; // 左右差精準度
  // 如果輪子沒有速度了話，則兩輪子用初始速度前進
  if (left_wheel_v == 0 || right_wheel_v == 0) go_forward(f_v);
  else{ 
    /*判斷車車是否走直線*/
    /*判斷車子左右的距離是否平均*/
    float dis_diff = distance[RIGHT] - distance[LEFT];
    if(dis_diff > acc){
      /*左偏 左輪給多一點速度*/
      /*如果速度太大則讓右輪速度減少*/
      if(left_wheel_v < v_max){
        go_left_moto(left_wheel_v + 1);
        return;
      }else if(right_wheel_v > f_v){
        go_right_moto(right_wheel_v - 1);
      }
    }else if(-dis_diff > acc){
      /*右偏 右輪給多一點速度*/
      /*如果速度太大則讓左輪速度減少*/
      if(right_wheel_v < v_max){
        go_right_moto(right_wheel_v + 1);
        return;
      }else if(left_wheel_v > f_v){
        go_left_moto(left_wheel_v - 1);
      }
    }
    /*走直線中 速度不變*/
    go_left_moto(left_wheel_v);      
    go_right_moto(right_wheel_v);
  }
  // float rad = detect_angle();
  // int a;

  // if (f_v>0) 
  //   a = 8;
  // else if(f_v<0)
  //   a = -8;
  // else
  //   a = 0;

  // if(rad == 0) {
  //   go_forward(f_v);
  // }
  // else if(rad > 0){
  //   go_right_moto(f_v+a);
  //   go_left_moto(f_v-a);
  // }else{
  //   go_right_moto(f_v-a);
  //   go_left_moto(f_v+a);
  // }
}

void go_forward(int v){

    go_right_moto(f_v);
    go_left_moto(f_v);
}


void go_stop(){
  if(go_forward_id != -1) timer.stop(go_forward_id);
  go_forward_id = -1;
  go_forward(0);
  end_time = millis();
  delay(80);
}

void go_left_moto(int v){
  left_wheel_v = v;  // 更新車子資料
  if (v > 0){
    analogWrite(Moto1, v);
    analogWrite(Moto2, 0);
  }else if(v < 0){
     analogWrite(Moto1, 0);
     analogWrite(Moto2, -v);
  }else{
     analogWrite(Moto1, 0);
     analogWrite(Moto2, 0);
  }
}

void go_right_moto(int v){  
  right_wheel_v = v; // 更新車子資料
  if (v > 0){
    analogWrite(Moto3, v);
    analogWrite(Moto4, 0);
  }else if(v < 0){
    analogWrite(Moto3, 0);
    analogWrite(Moto4, -v);
  }else{
     analogWrite(Moto3, 0);
     analogWrite(Moto4, 0);
  }
}

void update_dis(){
  // 更新距離
  for(int i=1;i<4;i++){
    distance[i] = dis(i);
  }
  // push進stack裡面
  s_dis dis;
  dis.right = distance[RIGHT];
  dis.front = distance[FRONT];
  dis.left = distance[LEFT];

  // if(!distance_data.isFull()){
    distance_data.push(dis);
  // }

}

void update_state(){
  if(no_barrier[LEFT] && no_barrier[FRONT] && no_barrier[RIGHT]){
    next_state = STATE_FRONT_RIGHT_LEFT;
  }else if(!no_barrier[LEFT] && no_barrier[FRONT] && !no_barrier[RIGHT]){
    next_state = STATE_FRONT;
  }else if(!no_barrier[LEFT] && !no_barrier[FRONT] && no_barrier[RIGHT]){
    next_state = STATE_RIGHT;
  }else if(no_barrier[LEFT] && !no_barrier[FRONT] && !no_barrier[RIGHT]){
    next_state = STATE_LEFT;
  }else if(!no_barrier[LEFT] && no_barrier[FRONT] && no_barrier[RIGHT]){
    next_state = STATE_FRONT_RIGHT;
  }else if(no_barrier[LEFT] && no_barrier[FRONT] && !no_barrier[RIGHT]){
    next_state = STATE_FRONT_LEFT;
  }else if(no_barrier[LEFT] && !no_barrier[FRONT] && no_barrier[RIGHT]){
    next_state = STATE_RIGHT_LEFT;
  }else if(!no_barrier[LEFT] && !no_barrier[FRONT] && !no_barrier[RIGHT]){
    next_state = STATE_DEAD;
  }else{
    next_state = 0;
  }
  if (now_state == next_state){
    state_change_flag = false;
  }else{
    state_change_flag = true;
    go_stop();  //交換state時，停下來
  }
  now_state = next_state;
}

void update_status(){
  s_dis first_dis;
  s_dis change[10] = {0,0,0,0,0,0,0,0,0,0};
  int i = -1;
  const float right_barrier_dis = 25;
  const float front_barrier_dis = 25;
  const float left_barrier_dis = 25;

  /************** 更新no_barrier *****************/
  // 如果stack裡面有十筆以上的資料了話，就更新資訊
  if (distance_data.count() >= 10){
    first_dis = distance_data.pop();
    // 把每一筆資料pop出來比對
    // change[]存每個值與第一個資料的差
    while(!distance_data.isEmpty()){
      s_dis next_dis;
      next_dis = distance_data.pop();

      if(i!=10){
        i++;
        change[i].right = next_dis.right - first_dis.right;
        change[i].front = next_dis.front - first_dis.front;
        change[i].front = next_dis.front - first_dis.front;
      }
    }      

    /* 先看right */
    // 與第一筆資料差+-5公分的有多少個
    int count_diff = 0;
    for(i=0;i<10;i++){
      if(change[i].right > 5 || change[i].right < -5){
        count_diff++;
      }
    }
    if(count_diff < 4){
      // 表示大部分跟第一筆資料差不多，第一筆資料為主，更新狀態
      // debug-------------------------------
      // Serial.print("update_right: ");
      // float time = millis();
      // Serial.print((time - start_time)/1000);
      // Serial.print("\n");
      // start_time = time;
      // debug-------------------------------
      no_barrier[RIGHT] = (first_dis.right > right_barrier_dis) ? true : false;
    }

    /* front */
    // 與第一筆資料差+-5公分的有多少個
    count_diff = 0;
    for(i=0;i<10;i++){
      if(change[i].front > 5 || change[i].front < -5){
        count_diff++;
      }
    }
    if(count_diff < 4){
      // 表示大部分跟第一筆資料差不多，第一筆資料為主，更新狀態
      // Serial.print("update_front\n");
      no_barrier[FRONT] = (first_dis.front > front_barrier_dis) ? true : false;
    }

    /* left */
    // 與第一筆資料差+-5公分的有多少個
    count_diff = 0;
    for(i=0;i<10;i++){
      if(change[i].left > 5 || change[i].left < -5){
        count_diff++;
      }
    }
    if(count_diff < 4){
      // 表示大部分跟第一筆資料差不多，第一筆資料為主，更新狀態
      // Serial.print("update_left\n");
      no_barrier[LEFT] = (first_dis.left > left_barrier_dis) ? true : false;
    }
  }
  /************** 更新next_state *****************/
  update_state();

}

float dis(int sensor){
  int trig, echo;
  // 判斷sensor trig echo pin腳  
  switch (sensor) {
      case RIGHT:
        trig = trig_1;
        echo = echo_1;
        break;
      case FRONT:
        trig = trig_2;
        echo = echo_2;
        break;
      case LEFT:
        trig = trig_3;
        echo = echo_3;
        break;
  }
  // 計算距離
  float duration = 0, distance = 0;
  int resen_count = 0;
  // 過濾訊號: 當距離偵測超過十公尺的時候，有可能有問題，重新偵測
  while(distance == 0 || distance > 1000 ){ 
    resen_count++;
    if (resen_count > 100) {
      /* 距離一直在預期之外 */
      break;
    }
    digitalWrite(trig, HIGH);
    delayMicroseconds(1000);
    digitalWrite(trig, LOW);
    duration = pulseIn (echo, HIGH);
    distance = (duration/2)/29.1;
  }
  //Serial.println(distance);
  return distance;
}


/************************以下為舊的code************************/
float detect_angle(){
  if (right_wheel_v == 0 || left_wheel_v == 0)
  return 0;
  float b_r = distance[RIGHT],
      b_l = distance[LEFT],
      b_f = distance[FRONT];
  const float acc = 0.05;     //精準度
  const float f = 5.5, l = 3.5, r = 3.5;
  float a_r = dis(RIGHT),
        a_l = dis(LEFT),
        a_f = dis(FRONT);
  float rad = 0;
  b_r = a_r - b_r;
  b_l = a_l - b_l;
  b_f = a_f - a_f;

  if((b_r <= acc && b_r >= -acc) && (b_l <= acc && b_l >= -acc)){
    //現在是走直線
    Serial.println("direct!");
  } else if(b_l > 0 || b_r < 0){
    //現在是斜向右
    rad = atan2(a_f + f, a_r + r);
    Serial.println("right!");
  } else if(b_r > 0 || b_l < 0){
    //現在是斜向左
    rad = -1 * atan2(a_f + f, a_l + l);
    Serial.println("left!");
  } else{
    Serial.println("???");
  }
  return rad;  
}