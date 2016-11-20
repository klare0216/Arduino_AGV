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
const int inter_time = 1000;
int now_f = 0;
bool now_direction[4]; //(0:none,1:right,2:forward,3:left)
bool now_dis[4]; //(0:none,1:right,2:forward,3:left)
int now_state;
int b_state;
int dead_flag = 0;
int next_state;
bool isStop = true;
int all_v = 73;
int all_v2 = 63;
int count = 0;
void d_go_3(int v);
void turn(float rad);
void turn_l(int v, int t); //左轉
void turn_r(int v, int t); //右轉
void plus_turn(int v, float rad);
void detect_f(); //偵測前方
void detect_lr(); //偵測側邊
void detect_stop();
void detect_l(); //偵測左方
float detect_angle(float b_f, float b_l, float b_r); //偵測偏倚角度
void update_direction(); 
void go(int v); //控制車子直線前進後退或停止
void go_l(int v); //控制左輪前進後退或停止
void go_r(int v); //控制右輪子前進後退或停止
void start_1();
void stop();
float dis(int trig, int echo); //回傳偵測到的距離
float dis(int trig, int echo, int acc); //回傳偵測到的距離(此值絕對小於acc)

void setup(){
  pinMode(Moto1, OUTPUT);
  pinMode(Moto2, OUTPUT); 
  pinMode(Moto3, OUTPUT);
  pinMode(Moto4, OUTPUT);
  analogWrite(Moto1, 0);
  analogWrite(Moto2, 0);
  analogWrite(Moto3, 0);
  analogWrite(Moto4, 0);
  Serial.begin(9600);
  pinMode (trig_1, OUTPUT);
  pinMode (echo_1, INPUT);
  pinMode (trig_2, OUTPUT);
  pinMode (echo_2, INPUT);
  pinMode (trig_3, OUTPUT);
  pinMode (echo_3, INPUT);
  //delay(3000);
}

void loop(){
  update_direction();
  detect_lr();
  start_1();
}
void start_1(){

  detect_stop();  // 避免卡住
  detect_f(); //避免前方撞牆
  detect_lr();
  switch(next_state){
    case 0:
      d_go_3(all_v);
      break;
    case 1:
      d_go_3(all_v);
      break;
    case 2:
      turn(-0.5);
      update_direction();
      if(now_direction[2] == 0){
        float f_b = dis(trig_2,echo_2);
        turn(-0.15);
        float f_a = dis(trig_2,echo_2);
        if(f_a > f_b){
          /*轉太多,左轉一點點向前*/
          turn(-0.25);
          d_go_3(all_v);
        }else{
          /*轉太少,右轉一點點向前*/
          turn(0.3);
          d_go_3(all_v);
        }
      }
      break;
    case 3:
      turn(0.5);      
      update_direction();
      if(now_direction[2] == 0){
        float f_b = dis(trig_2,echo_2);
        turn(0.15);
        float f_a = dis(trig_2,echo_2);
        if(f_a > f_b){
          /*轉太多,右轉一點點向前*/
          turn(0.25);
          d_go_3(all_v);
        }else{
          /*轉太少,左轉一點點向前*/
          turn(-0.3);
          d_go_3(all_v);
        }
      }
      break;
    case 4:
      d_go_3(all_v);
      break;
    case 5:
      d_go_3(all_v);
      break;
    case 6:
      b_state = 6;
      update_direction();
      next_state = (now_dis[1] > now_dis[3])?2:3;
      start_1();
      break;
    case 7:
      /*判斷是否是死路迴轉的*/
      if(dead_flag == 1){
        /*判斷前一個岔路長什麼樣子*/
        switch(b_state){
          case 6:
            next_state = 1;
            start_1();
            break;
          case 8:
            next_state = 2;
            start_1();
            break;
        }
        /*死路走出來了!*/
        dead_flag = 0;
      }else{
        b_state = 7;
        next_state = 1;
        start_1();
      }
      break;
    case 8:
      /*判斷是否是死路迴轉的*/
      if(dead_flag == 1){
        /*判斷前一個岔路長什麼樣子*/
        switch(b_state){
          case 6:
            next_state = 1;
            start_1();
            break;
          case 7:
            next_state = 3;
            start_1();
            break;
        }
        /*死路走出來了!*/
        dead_flag = 0;
      }else{
        b_state = 8;
        next_state = 1;
        start_1();
      }
      break;
    case 9:
      // 判斷是否是死路
      update_direction();
      if(now_dis[1] > now_dis[3]){
        /*先右轉一點*/
        turn(-0.15);
      }else{
        /*先左轉一點*/
        turn(0.15);
      }
      update_direction();
      /*判斷是否還是state 9*/
      if(now_state == 9){
        /*迴轉出來*/
        turn_l(all_v, 1);
        dead_flag = 1;
      }
      break;
    default:
      go(0);
  }
}



void detect_lr(){
  float right_distance = dis(trig_1,echo_1);
  float laft_distance = dis(trig_3,echo_3);
  int d_min = 5;
  
  if (right_distance > d_min && laft_distance > d_min) //沒有快撞到東西 
    return;
  else{
    //while(right_distance < d_min || laft_distance < d_min){
      detect_f();
      if (right_distance < d_min) {
        d_go_3(-all_v);
        delay(200);
        turn(0.13);
        delay(100);
        d_go_3(all_v);
        delay(100);
      }else{ 
        d_go_3(-all_v);
        delay(200);
        turn(-0.13);
        d_go_3(all_v);
        delay(100);
      }
      right_distance = dis(trig_1,echo_1);
      laft_distance = dis(trig_3,echo_3);
    //}
    stop();
  }
}

void update_direction(){
  
  int f = dis(trig_2, echo_2), r = dis(trig_1, echo_1), l = dis(trig_3, echo_3);
  now_dis[1] = r;
  now_dis[2] = f;
  now_dis[3] = l;
  now_direction[1] = (r < 28)? false : true;
  now_direction[2] = (f < 10)? false : true;
  now_direction[3] = (l < 28)? false : true;
  Serial.print(" ");
  Serial.print(r);
  Serial.print(" ");
  Serial.print(f);
  Serial.print(" ");
  Serial.println(l);
  if(!now_direction[1] && now_direction[2] && !now_direction[3])
    next_state = 1;
  else if(now_direction[1] && !now_direction[2] && !now_direction[3])
    next_state = 2;
  else if(!now_direction[1] && !now_direction[2] && now_direction[3])
    next_state = 3;
  else if(now_direction[1] && now_direction[2] && !now_direction[3])
    next_state = 4;
  else if(!now_direction[1] && now_direction[2] && now_direction[3])
    next_state = 5;
  else if(now_direction[1] && !now_direction[2] && now_direction[3])
    next_state = 6;
  else if(now_direction[1] && now_direction[2] && !now_direction[3])
    next_state = 7;
  else if(!now_direction[1] && now_direction[2] && now_direction[3])
    next_state = 8;
  else if(!now_direction[1] && !now_direction[2] && !now_direction[3])
    next_state = 9;
  delay(80);  
}

void plus_turn(int v, float rad){
  int a;

  if (v>0) 
    a = 8;
  else if(v<0)
    a = -8;
  else
    a = 0;

  if(rad == 0) 
    go(v);
  else if(rad > 0){
    go_r(v+a);
    go_l(v-a);
  }else{
    go_r(v-a);
    go_l(v+a);
  }
}

void turn(float rad){
  if(rad > 0)
    turn_l(80, rad*700);
  else if(rad < 0)
    turn_r(70, -rad*700);
}

void detect_stop(){
  float front_distance = dis(trig_2,echo_2);
  float acc = 1;
  float d_min = 6;
  if((front_distance - now_f) < acc && (front_distance - now_f) > -acc){
    Serial.println("STOP!!");
    while(front_distance < d_min){
      stop();
      int rad = detect_angle(front_distance, dis(trig_3,echo_3), dis(trig_1,echo_1));
      plus_turn(-all_v2, rad);
      delay(80);   
      front_distance = dis(trig_2,echo_2); 
    }
    stop();
  }
  now_f = dis(trig_2,echo_2);
}

void detect_f(){
  float front_distance = dis(trig_2,echo_2);
  int d_min = 8;
  
  if (front_distance > d_min) //沒有快撞到東西 
    return;
  else{
      stop();
    while(front_distance < d_min){
      Serial.println("detect_f! backward!");
      float rad = detect_angle(front_distance, dis(trig_3,echo_3), dis(trig_1,echo_1));
      plus_turn(-all_v2, rad);
      delay(100);   
      front_distance = dis(trig_2,echo_2); 
    }
    stop();
  }
}

void turn_l(int v, int t){
  go_l(-v);
  go_r(v);
  delay(t);
  go(0);
}

void turn_r(int v, int t){
  go_l(v);
  go_r(-v);
  delay(t);
  go(0);
}

void stop(){
  go(0);
  delay(100);
}


void d_go_3(int v){
  float b_r, b_l, b_f; //右左前 前一次的距離
  int d = b_r - b_l;
  const float acc = 0.05;     //精準度
  const float f = 5.5, l = 3.5, r = 3.5;
  
  go(v);
  b_r = dis(trig_1,echo_1);
  b_l = dis(trig_3,echo_3);
  b_f = dis(trig_2,echo_2);
  delay(all_v2);
  float rad = detect_angle(b_f, b_l, b_r);

  if(rad>0){
    if(d < -10){
      go(-all_v2);
      delay(300);
      stop();
      turn(rad);
    }else
      plus_turn(v, rad);
  }else if(rad<0){
    if(d > 10){
      go(-all_v2);
      stop();
      delay(300);
      turn(rad);
    } else{
      plus_turn(v, rad);
    }
  }
}


float detect_angle(float b_f, float b_l, float b_r){
  const float acc = 0.05;     //精準度
  const float f = 5.5, l = 3.5, r = 3.5;
  float a_f, a_l, a_r;
  a_r = dis(trig_1,echo_1);
  a_l = dis(trig_3,echo_3);
  a_f = dis(trig_2,echo_2);
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


float dis(int trig, int echo, int acc){
  float duration = 0, distance = 0;
  while(distance == 0 || distance > acc){ 
    digitalWrite(trig, HIGH);
    delayMicroseconds(1000);
    digitalWrite(trig, LOW);
    duration = pulseIn (echo, HIGH);
    distance = (duration/2)/29.1;
  }
  //Serial.println(distance);
  return distance;
}

float dis(int trig, int echo){
  float duration = 0, distance = 0;
  while(distance == 0){ 
    digitalWrite(trig, HIGH);
    delayMicroseconds(1000);
    digitalWrite(trig, LOW);
    duration = pulseIn (echo, HIGH);
    distance = (duration/2)/29.1;
    
  }
  //Serial.println(distance);
  return distance;
}

void go_l(int v){
  if (v > 0){
    analogWrite(Moto1, v);
    analogWrite(Moto2, 0);
    isStop = false;
  }else if(v < 0){
     analogWrite(Moto1, 0);
     analogWrite(Moto2, -v);
     isStop = false;
  }else{
     analogWrite(Moto1, 0);
     analogWrite(Moto2, 0);
     isStop = true;
  }
}

void go_r(int v){
  if (v > 0){
    analogWrite(Moto3, v);
    analogWrite(Moto4, 0);
    isStop = false;
  }else if(v < 0){
    analogWrite(Moto3, 0);
    analogWrite(Moto4, -v);
    isStop = false; 
  }else{
     analogWrite(Moto3, 0);
     analogWrite(Moto4, 0);
     isStop = true;
  }
}

void go(int v){
   go_r(v);
   go_l(v);
}
