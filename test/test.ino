#include <Event.h>
#include <Timer.h>
#include <StackArray.h>
#include <BRCClient.h>
/*--------------wifi message------------------*/

#if !defined(UNO) && defined(USE_HARDWARE_SERIAL)
 BRCClient brcClient(&HW_SERIAL);
#else
 BRCClient brcClient(UART_RX, UART_TX);
#endif
// You have to modify the corresponding parameter
#define AP_SSID    "AP_SSID"
#define AP_PASSWD  "AP_PASSWD"
#define TCP_IP     "TCP_IP"
#define TCP_PORT   5000
#define MY_COMM_ID 0x24
#define PARTNER_COMM_ID 0x20
/*------------------------------------------*/

const int enable_left = 12;
const int enable_right = 13;
const int Moto1 = 5;
const int Moto2 = 4;
const int Moto3 = 6;
const int Moto4 = 7;
const int echo_1 = 26;
const int trig_1 = 27;
const int echo_2 = 24;
const int trig_2 = 25;
const int echo_3 = 22;
const int trig_3 = 23;
Timer timer;
typedef struct RFL_dis{
  float right;
  float front;
  float left;
} s_dis;
/*--------------可以使用的數值------------------*/
const bool f_v = true;
const int f_l_v = 200;                 // 前進速度
const int f_r_v = 235;
const int v_max = 120;              // 最大速度
const int f_delay = 950;
const int state_change_count_const = 5;
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
# define UART_RX 10
# define UART_TX 2
/*----------------車子現況資料-----------------*/
int right_wheel_v = 0;              // 右輪速度
int left_wheel_v = 0;               // 左輪速度
float distance[4] = {0, 0, 0, 0};   // 三顆sensor的距離
bool no_barrier[4] = {1, 1, 1, 1};  // 有障礙與否
int next_state = 0;                 // 下一個的狀態
int now_state = 0;                  // 現在的狀態
StackArray <s_dis> distance_data;   // 暫存15筆偵測資料
int state_change_count = 0;      // 當此count == state_change_count_const ,call next_step
int go_forward_id = -1;
int detect_block_id = -1;
int start_f_dis = 0;
int diff_f_dis = 0;
int count_block = 0;
bool dead_flag = false;         // 是否走死路
int count_left = 0;
/*----------------地圖資訊---------------------*/
// int mapp[11][11] = {
//   (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
//   (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
//   (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
//   (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
//   (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
//   (0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0),
//   (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
//   (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
//   (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
//   (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
//   (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
// }
StackArray <int> path_history;   // 紀錄路徑
/*---------------function 宣告-----------------*/
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
void go_turn_nonstop(int degree); // 旋轉不停
void go_forward();                  // 前進 with v=f_v
void go_forward(int v);             // 前進
void go_backward(int v);             // 後退進
void go_stop();                     // 停止
void go_left_moto(int v);           // 左輪前進
void go_right_moto(int v);          // 右輪前進
void update_detect_state();         // 為了來偵測狀態的更新
void update_dis();                  // 更新距離
void update_nextstate();            // 更新下一次狀態
void update_nowstate();             // 更新現在狀態
void update_nowstate_nonstop();     // 更新狀態，當nowstate改變的時候不會停下
void update_status();               // 更新車子現況資料
void update_message();
void detect_front();                // 與前方距離xcm才停下來
void detect_block();                // 偵測是否走了一格
float dis(int sensor);              // 回傳sensor測到的距離
void send_message();
void debug();

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
  // 連上基地台
  while (!Serial)
    ;

  brcClient.begin(9600);
  brcClient.beginBRCClient(AP_SSID, AP_PASSWD, TCP_IP, TCP_PORT);

  delay(2000);
  if (brcClient.registerID(MY_COMM_ID))
    Serial.println("ID register OK");
  else {
    Serial.println("ID register FAIL");
    brcClient.endBRCClient();

    while (1)
      ;
  // 初始化
  now_state = STATE_FRONT;
 timer.every(1000,writeToSerial);
 timer.every(1,car_loop);
   // timer.every(1000,debug);
}

void debug(){

  // go_right_moto(235);
  // go_left_moto(200);
  Serial.print(dis(RIGHT));
  Serial.print(" ");
  Serial.print(dis(FRONT));
  Serial.print(" ");
  Serial.print(dis(LEFT));
  Serial.print("\n");

}

void loop(){
  timer.update();
}

void car_loop(){
  update_message();
  update_dis();     // 更新目前距離
  update_status();
  update_nextstate();
  update_nowstate();
  if (state_change_count == state_change_count_const) next_step(); // 狀態改變的時候，執行下一步
  // next_step();
}

void writeToSerial(){
  Serial.print("distance: ");
  for(int i = 1;i<4;i++){
    Serial.print(distance[i]);
    Serial.print(" ");
  }
  Serial.print("    ");
  // Serial.print("block_count: ");
  // Serial.print(block_count);
  // Serial.print("  ");
  // Serial.print("oneblock_last_fdis: ");
  // Serial.print(oneblock_last_fdis);
  // Serial.print("  ");
  Serial.print("no_barrier: ");
  for(int i = 1;i<4;i++){
    Serial.print(no_barrier[i]);
    Serial.print(" ");
  }
  // Serial.print("distance_data_count: ");
  // Serial.print(distance_data.count());
  // Serial.print(" start_time: ");
  // Serial.print(start_time+200);
  // Serial.print(" now_time: ");
  // Serial.print(millis());
  Serial.print(" state_change_flag:");
  Serial.print(state_change_count);
  Serial.print(" state: ");
  Serial.print(next_state);
  Serial.println("");
}

void next_step(){

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
      go_stop();
  }
}

void step_front(){
  /*紀錄開始距離*/
  update_dis();
  start_f_dis = distance[FRONT]; 
  Serial.print("s_f: ");
  Serial.println(start_f_dis);
  /*持續呼叫go_forward*/
  if(go_forward_id == -1){
    go_forward_id = timer.every(1,go_forward);  
  }
  /*持續偵測是否走一格*/
  if(detect_block_id == -1){
    detect_block_id = timer.every(1,detect_block);
  }
}

void step_right(){
  int acc = 15;
  /* 前進到剩下acc */
  while(dis(FRONT) > acc){
    go_forward(f_v);
  }
  go_stop();
  /*旋轉順時鐘九十度*/
  /*此時的狀態應該要是STATE_FRONT_RIGHT 由此判定是否轉對*/
  while(dis(LEFT)>30||dis(FRONT)<30||dis(RIGHT)<30){
    go_turn(-65); //更新狀態
    if(dis(FRONT)>=30){
      go_forward(f_v);
      delay(300);
      go_stop();
      if(dis(FRONT)<30){
        go_forward(-f_v);
        delay(300);
        go_stop();
        continue;
      }
    }
  }
  go_stop();
  /*前進30cm*/
  go_forward(f_v);
  delay(f_delay);
  // go_stop();
}

void step_left(){
  int acc = 15;
  /* 前進到剩下acc */
  while(dis(FRONT) > acc){
    go_forward(f_v);
  }
  go_stop();
  /*旋轉逆時鐘九十度*/
  go_turn(65);
  bool front_is_empty = true;
  while(dis(LEFT)<30||dis(FRONT)<30||dis(RIGHT)>30||!front_is_empty){
    detect_front();
    go_turn(30);
    if(dis(FRONT)>=30){
      go_forward(f_v);
      delay(300);
      go_stop();
      front_is_empty = (dis(FRONT)<30)?false:true;  
      go_forward(-f_v);
      delay(300);
      go_stop();
    }
  }
  count_left++;
  /*前進30cm*/
  go_forward(f_v);
  delay(f_delay); 
}

void step_front_right(){
  go_forward(f_v);
  delay(200);
  go_stop();
  /*先右轉*/
  /*旋轉順時鐘九十度*/
  bool front_is_empty = true;
  go_turn(-65);
  while(dis(LEFT)<30||dis(FRONT)<30||dis(RIGHT)<30||!front_is_empty){
    detect_front();
    go_turn(-25);
    if(dis(FRONT)>=30){
      go_turn(-20);
      go_forward(f_v);
      delay(300);
      go_stop();
      front_is_empty = (dis(FRONT)<30)?false:true;  
      go_forward(-f_v);
      delay(320);
      go_stop();
    }
  }
  /*前進*/
  go_forward(f_v);
  delay(f_delay);
}

void step_front_left(){
  go_forward(f_v);
  delay(200);
  go_stop();
  /*先左轉*/
  /*旋轉逆時鐘九十度*/
  bool front_is_empty = true;
  go_turn(65);
  while(dis(LEFT)<30||dis(FRONT)<30||dis(RIGHT)<30||!front_is_empty){
    go_turn(30);
    if(dis(FRONT)>=30){
      go_turn(20);
      go_forward(f_v);
      delay(300);
      go_stop();
      front_is_empty = (dis(FRONT)<30)?false:true;  
      go_forward(-f_v);
      delay(300);
      go_stop();
    }
  }
  count_left++;
  /*前進*/
  go_forward(f_v);
  delay(f_delay);
}

void step_right_left(){
  if(count_left>=4){
    dead_flag = true;
  }
  int acc = 15;
    /* 前進到剩下acc */
  while(dis(FRONT) > acc){
    go_forward(f_v);
  }
  go_stop();
  go_forward(f_v);
  delay(200);
  /*旋轉順時鐘九十度*/
  if(!dead_flag){
    /*左轉*/
    bool front_is_empty = true;
    go_turn(65);
    while(dis(LEFT)<30||dis(FRONT)<30||dis(RIGHT)>30||!front_is_empty){
      go_turn(30);
      if(dis(FRONT)>=30){
        go_turn(20);
        go_forward(f_v);
        delay(300);
        go_stop();
        front_is_empty = (dis(FRONT)<30)?false:true;  
        go_forward(-f_v);
        delay(300);
        go_stop();
      }
    }
    count_left++;
  }else{
   /*右轉*/
    bool front_is_empty = true;
    go_turn(-65);
    while(dis(LEFT)>30||dis(FRONT)<30||dis(RIGHT)<30||!front_is_empty){
      go_turn(-30);
      if(dis(FRONT)>=30){
        go_turn(-20);
        go_forward(f_v);
        delay(300);
        go_stop();
        front_is_empty = (dis(FRONT)<30)?false:true;  
        go_forward(-f_v);
        delay(300);
        go_stop();
      }
      dead_flag = false;
    } 
  }
  /*前進*/
  go_forward(f_v);
  delay(f_delay);
}

void step_dead(){
  /*停在牆壁前面*/
  int acc = 10;
  while(dis(FRONT) > acc){
    go_forward(f_v);
  }
  go_stop();
  update_nowstate_nonstop();
  if(now_state!=STATE_DEAD) return;
  /*旋轉逆時鐘一百八十度*/
  if(dis(RIGHT)>dis(LEFT)){
    /*順時針轉*/
    bool front_is_empty = true;
    go_turn(-150);
    while(dis(LEFT)>28||dis(FRONT)<30||dis(RIGHT)>28||!front_is_empty){
      detect_front();
      go_turn(-30);
      if(dis(FRONT)>=30){
        go_turn(-20);
        go_forward(f_v);
        delay(300);
        go_stop();
        front_is_empty = (dis(FRONT)<30)?false:true;  
        go_forward(-f_v);
        delay(300);
        go_stop();
      }
    }
  }else{
    /*逆時針轉*/
    bool front_is_empty = true;
    go_turn(150);
    while(dis(LEFT)>28||dis(FRONT)<30||dis(RIGHT)>28){
      go_turn(30);
      if(dis(FRONT)>=30){
        go_turn(-20);
        go_forward(f_v);
        delay(300);
        go_stop();
        front_is_empty = (dis(FRONT)<30)?false:true;  
        go_forward(-f_v);
        delay(300);
        go_stop();
      }
    }
  }
  go_forward(f_v);
  delay(f_delay);

  dead_flag = true;
}

void go_turn(float degree){
  if(degree > 0){
    /*逆時針*/
    go_left_moto(-225);
    go_right_moto(225);
    delay((float)398/90*degree);
    go_stop();
  }else if(degree < 0){
    /*順時針*/
    go_left_moto(225);
    go_right_moto(-225);
    delay((float)393/90*(-degree));
    go_stop();
  }
}

void go_turn_nonstop(int degree){
  if(degree > 0){
    /*逆時針*/
    go_left_moto(-225);
    go_right_moto(225);
  }else if(degree < 0){
    /*順時針*/
    go_left_moto(225);
    go_right_moto(-225);
  }
}

void go_forward(){
  int acc = 5; // 左右差精準度
  // 如果輪子沒有速度了話，則兩輪子用初始速度前進
  if (left_wheel_v == 0 || right_wheel_v == 0) go_forward(f_v);
  else{ 
    /*判斷車車是否走直線*/
    if (distance[RIGHT]+distance[LEFT] > 35){
      go_left_moto(left_wheel_v);      
      go_right_moto(right_wheel_v);
      return;
    }
    float dis_diff = distance[RIGHT] - distance[LEFT];
    if(dis(RIGHT)<=7){
    /*如果太靠近牆壁*/
    /*左後退*/
      go_right_moto(-90);
      go_left_moto(-70);
      delay(350);
      go_stop();
      go_turn(25);
      go_forward(f_v);
    }else if(dis(LEFT)<=7){
    /*如果太靠近牆壁*/
    /*左後退*/
      go_right_moto(-70);
      go_left_moto(-90);
      delay(350);
      go_stop();
      go_turn(-25);
      go_forward(f_v);
    /****************判斷車子是否太靠近牆壁****************/
    }else if(distance[RIGHT] < 10){
      go_left_moto(f_v - 2);
      go_right_moto(f_v + 2);
      delay(50);
      go_left_moto(f_v + 1);
      go_right_moto(f_v - 1);
      delay(10);
      go_forward(f_v);
      return;
    }else if(distance[LEFT] < 6){
      go_left_moto(f_v - 2);
      go_right_moto(f_v + 2);
      delay(50);
      go_left_moto(f_v - 1);
      go_right_moto(f_v + 1);
      delay(10);
      go_forward(f_v);
      return;
    }
    /****************判斷車子左右的距離是否平均****************/
    else if(dis_diff > acc){
      /*左偏 左輪給多一點速度*/
      /*如果速度太大則讓右輪速度減少*/
      if(left_wheel_v < v_max){
        go_left_moto(left_wheel_v + 1);
        return;
      }else if(right_wheel_v > f_v){
        go_right_moto(right_wheel_v - 1);
        return;
      }
    }else if(-dis_diff > acc){
      /*右偏 右輪給多一點速度*/
      /*如果速度太大則讓左輪速度減少*/
      if(right_wheel_v < v_max){
        go_right_moto(right_wheel_v + 1);
        return;
      }else if(left_wheel_v > f_v){
        go_left_moto(left_wheel_v - 1);
        return;
      }
    }else{
      /**********************判斷車子是否是斜的走********************/  
      float d_r = distance[RIGHT], d_l = distance[LEFT];
      go_forward(f_v);
      delay(10);
      update_dis();
      if(d_r<distance[RIGHT]&&d_l>distance[LEFT]){
        /*車子左偏*/
        /*轉成正的*/
        go_turn_nonstop(-1);
        delay(15);
        go_forward(f_v);
      }else if(d_r>distance[RIGHT]&&d_l<distance[LEFT]){
        /*車子右偏*/
        /*轉成正的*/
        go_turn_nonstop(-1);
        delay(15);
        go_forward(f_v);
      }
      else{
      /****************走直線中 速度不變****************/
        go_left_moto(left_wheel_v);      
        go_right_moto(right_wheel_v);
      }
    }
  }
}

void go_forward(int v){
  if (v == 1){
    go_left_moto(f_l_v);
    go_right_moto(f_r_v);
  }else{
    go_left_moto(v);
    go_right_moto(v);
  }
}

void go_backward(int v){
  if (v == 1){
    go_left_moto(-f_l_v);
    go_right_moto(-f_r_v);
  }else{
    go_left_moto(-v);
    go_right_moto(-v);
  }
}

void go_stop(){
  if(go_forward_id != -1) timer.stop(go_forward_id);
  if(detect_block_id != -1) timer.stop(detect_block_id);
  detect_block_id = -1;
  go_forward_id = -1;
  go_forward(0);
  /*紀錄結束點*/
  // update_dis();
  // if(start_f_dis!=0){
  //   end_f_dis = start_f_dis-distance[front];
  //   start_f_dis = 0;
  // }
  delay(80);
}

void go_left_moto(int v){
  digitalWrite(enable_left, HIGH);
  left_wheel_v = v;  // 更新車子資料
  if (v > 0){
    analogWrite(Moto1, v);
    analogWrite(Moto2, 0);
  }else if(v < 0){
    analogWrite(Moto1, 0);
    analogWrite(Moto2, -v);
  }else{
    digitalWrite(enable_left, LOW);
    analogWrite(Moto1, 0);
    analogWrite(Moto2, 0);
  }
}

void go_right_moto(int v){  
  digitalWrite(enable_right, HIGH);
  right_wheel_v = v; // 更新車子資料
  if (v > 0){
    analogWrite(Moto3, v);
    analogWrite(Moto4, 0);
  }else if(v < 0){
    analogWrite(Moto3, 0);
    analogWrite(Moto4, -v);
  }else{
    digitalWrite(enable_right, LOW);
    analogWrite(Moto3, 0);
    analogWrite(Moto4, 0);
  }
}

void update_message(){

}

void update_detect_state(){
  update_dis();
  update_status();
  update_nextstate();
  update_nowstate_nonstop();
}

void update_dis(){
  // // 更新距離
  for(int i=1;i<4;i++){
    distance[i] = dis(i);
  }
  // // push進stack裡面
  s_dis dis;
  dis.right = distance[RIGHT];
  dis.front = distance[FRONT];
  dis.left = distance[LEFT];

  distance_data.push(dis);

}

void update_nextstate(){
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
}

void update_nowstate(){
  if (now_state == next_state){
    state_change_count ++;
  }else{
    state_change_count = 0;
    go_stop();
  }
  now_state = next_state;
}

void update_nowstate_nonstop(){
  if (now_state == next_state){
    state_change_count ++;
  }else{
    state_change_count = 0;
  }
  now_state = next_state;
}

void update_status(){
  s_dis dis[5] = {0,0,0,0,0};
  s_dis change[5];
  const float right_barrier_dis = 25;
  const float front_barrier_dis = 25;
  const float left_barrier_dis = 25;
  int const acc = 30;
  /************* 對照資料 **************/
  // Serial.println(distance_data.count());
  if(distance_data.count() >= 5){
    for(int i = 0;i<5;i++){
      change[i] = dis[i] = distance_data.pop();
    }
    /***************處理中間與其他的值的差****************/
    /*left*/
    int count_diff = 0; // 數與中間差異太大數量
    // Serial.print("dis.left: ");
    for(int i = 0;i<5;i++){
      // Serial.print(change[i].left);
      // Serial.print(" ");
      change[i].left -= dis[3].left;
      if (change[i].left > acc || change[i].left < -acc) count_diff++;
    }
    // Serial.print("count_diff: ");
    // Serial.print(count_diff);
    // Serial.print("\n");
    /*如果差異不大才算是正確資料*/
    if(count_diff == 0){
        no_barrier[LEFT] = (dis[3].left > left_barrier_dis) ? true : false;
    }
    /*right*/
    count_diff = 0; // 數與中間差異太大數量
    // Serial.print("dis.right: ");
    for(int i = 0;i<5;i++){
      // Serial.print(change[i].right);
      // Serial.print(" ");
      change[i].right -= dis[3].right;
      if (change[i].right > acc || change[i].right < -acc) count_diff++;
    }
    // Serial.print("count_diff: ");
    // Serial.print(count_diff);
    // Serial.print("\n");

    /*如果差異不大才算是正確資料*/
    if(count_diff == 0){
        no_barrier[RIGHT] = (dis[3].right > right_barrier_dis) ? true : false;
    }
    /*front*/
    count_diff = 0; // 數與中間差異太大數量
    // Serial.print("dis.front: ");
    for(int i = 0;i<5;i++){
      // Serial.print(change[i].front);
      // Serial.print(" ");
      change[i].front -= dis[3].front;
      if (change[i].front > acc || change[i].front < -acc) count_diff++;
    }
    // Serial.print("count_diff: ");
    // Serial.print(count_diff);
    // Serial.print("\n");

    /*如果差異不大才算是正確資料*/
    if(count_diff == 0){
        no_barrier[FRONT] = (dis[3].front > front_barrier_dis) ? true : false;
    }
      /*把新的四個資料Push回去*/
    distance_data.push(dis[3]);
    distance_data.push(dis[2]);
    distance_data.push(dis[1]);
    distance_data.push(dis[0]);
  }
}
void detect_front(){
  while(dis(FRONT)<14){
    go_forward(-f_v);
  }
  go_stop();
}

void detect_block(){
  update_dis();
  float acc = 2;
  float diff = (start_f_dis - distance[FRONT])-30;
  if(diff < acc && diff > -acc){
    Serial.print("one block: ");
    Serial.println(diff);
    /*大約走了一格*/
    count_block++;
    /*重置前方距離*/
    start_f_dis = distance[FRONT];
    go_stop();
    delay(2000);
    /*重置state_change_count，使下一次偵測能夠再度進到next_step()*/
    state_change_count = 0;
    
  }
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

void send_message(){
  brcClient.sendToClient(PARTNER_COMM_ID, "Hello");
  delay(2000);
  brcClient.broadcast("World");
  delay(2000);
  // brcClient.endBRCClient();
}