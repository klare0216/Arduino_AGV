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
/* 可以使用的數值 */
const int f_v; // 前進速度
/* 車子現況資料 */
int right_wheel_v = 0; //左輪速度
int left_wheel_v = 0; //右輪速度

void go_forward(int v); // 前進
void go_left_moto(int v); // 左輪前進
void go_right_moto(int v); // 右輪前進

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
}


void go_forward(int v){

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
  }}