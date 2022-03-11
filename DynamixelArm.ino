// https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/
// https://emanual.robotis.com/docs/en/parts/controller/opencm904/
// https://emanual.robotis.com/docs/en/software/arduino_ide/#library-api

#include <Dynamixel2Arduino.h>  // Подключение библиотеки Dynamixel
#include <math.h>

#define DEBUG_SERIAL Serial // Установка константы, отвечающей за последовательный порт, подключаемый к компьютеру
#define DXL_SERIAL Serial3 // OpenCM9.04 EXP Board's DXL port Serial. (To use the DXL port on the OpenCM 9.04 board, you must use Serial1 for Serial. And because of the OpenCM 9.04 driver code, you must call Serial1.setDxlMode(true); before dxl.begin();.)

#define DXL_DIR_PIN 22 // Инициализация переменной, отвечащей за номер пина, подключенного к информационному пину приводов манипулятора
#define DXL_PROTOCOL_VERSION 1.0 // Инициализация переменной, отвечащей за протокол передачи данных от OpenCM9.04 к приводам
#define JOINT_N 6 // Количество приводов манипулятора
#define DYNAMIXEL_GOAL_POS_ERROR 5 // Погрешность позиции для динимикселей

#define EXP_BOARD_BUTTON1_PIN 16 // Пин кнопки 1 на плате расширения
#define EXP_BOARD_BUTTON2_PIN 17 // Пин кнопки 2 на плате расширения
#define EXP_BOARD_LED1_PIN 18 // Пин светодиода 1 на плате расширения
#define EXP_BOARD_LED2_PIN 19 // Пин светодиода 2 на плате расширения
#define EXP_BOARD_LED3_PIN 20 // Пин светодиода 3 на плате расширения

// Длины звеньев
#define LINK1 103
#define LINK2 105
#define LINK3 89
#define LINK4 168

// Для светодиодов на плате расширения, которые от земли
#define LED_HIGH LOW
#define LED_LOW HIGH

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN); // Инициализация указателя на команды из библиотеки Dynamixel

void setup() {
  DEBUG_SERIAL.begin(57600); // Установка скорости обмена данными по последовательному порту компьютера
  pinMode(EXP_BOARD_BUTTON1_PIN, INPUT_PULLDOWN); // Установка режима кнопки 1 на плате расширения
  pinMode(EXP_BOARD_BUTTON2_PIN, INPUT_PULLDOWN); // Установка режима кнопки 2 на плате расширения
  pinMode(EXP_BOARD_LED1_PIN, OUTPUT); // Установка режима пина светодиода 1 на плате расширения
  pinMode(EXP_BOARD_LED2_PIN, OUTPUT); // Установка режима пина светодиода 2 на плате расширения
  pinMode(EXP_BOARD_LED3_PIN, OUTPUT); // Установка режима пина светодиода 3 на плате расширения
  digitalWrite(EXP_BOARD_LED1_PIN, LED_LOW); // Выключаем светодиод 1 на плате расширения
  digitalWrite(EXP_BOARD_LED2_PIN, LED_LOW); // Выключаем светодиод 2 на плате расширения
  digitalWrite(EXP_BOARD_LED3_PIN, LED_LOW); // Выключаем светодиод 3 на плате расширения
  //while(!DEBUG_SERIAL); // Ждём, пока монитор порта не откроется
  while(digitalRead(EXP_BOARD_BUTTON1_PIN) == 0); // Ждём, пока не будет нажата кнопка 1 на плате расширения
  DEBUG_SERIAL.println("Setup...");
  dxl.begin(1000000); // Установка скорости обмена данными по последовательному порту манипулятора
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION); // Выбор протокола обмена данными
  for (byte i = 1; i <= JOINT_N; i++) { // Цикл для перебора всех приводов
    while (true) {
      if(dxl.ping(i) == true) { // Проверка отвечает ли мотор
        DEBUG_SERIAL.print("Dynamixel with ID "); DEBUG_SERIAL.print(i); DEBUG_SERIAL.print(" found, model "); DEBUG_SERIAL.print(dxl.getModelNumber(i)); DEBUG_SERIAL.println(".");
        break;
      } else {
        DEBUG_SERIAL.print("Dynamixel with ID "); DEBUG_SERIAL.print(i); DEBUG_SERIAL.print(" not found!"); DEBUG_SERIAL.println(" Wait...");
        delay(500);
      }
    }
    dxl.torqueOff(i); // Отключение блокировки привода, чтобы установить режим работы!
    bool setDinamixelOperationMode = dxl.setOperatingMode(i, OP_POSITION); // Установка режима работы привода в качестве шарнира
    if (!setDinamixelOperationMode) {
      DEBUG_SERIAL.print("Dynamixel wgith ID "); DEBUG_SERIAL.print(i); DEBUG_SERIAL.println("mode not set!");
    }
    delay(10);
  }
  DEBUG_SERIAL.println("Start..."); DEBUG_SERIAL.println();
  delay(500);
  // Занять среднюю позицию
  int motorsGoalPos[] = {512, 512, 512, 512, 512, 512};
  for (byte i = 1; i <= JOINT_N; i++) {
    MoveMotorToGoal(i, 50, motorsGoalPos[i]);
  }
  //int* motGoalPos = new int[6];
  //motGoalPos = GetMotorsGoalPos();
  //WaitMotorsTakeGoalPos(motGoalPos); // Ждём, чтобы все приводы заняли позицию
  WaitMotorsTakeGoalPos();
  delay(500);
}

void loop() {
  /*float* motPos = new float[3];
  motPos = Delta_IK(0, 50, -180);
  DEBUG_SERIAL.print("NeedMotorPos: "); DEBUG_SERIAL.print(motPos[0]); DEBUG_SERIAL.print(", "); DEBUG_SERIAL.print(motPos[1]); DEBUG_SERIAL.print(", "); DEBUG_SERIAL.println(motPos[2]);
  motPos[0] = ConvertDegreesToGoalPos(motPos[0]);
  motPos[1] = ConvertDegreesToGoalPos(motPos[1]);
  motPos[2] = ConvertDegreesToGoalPos(motPos[2]);
  DEBUG_SERIAL.print("NeedGoalPos: "); DEBUG_SERIAL.print(motPos[0]); DEBUG_SERIAL.print(", "); DEBUG_SERIAL.print(motPos[1]); DEBUG_SERIAL.print(", "); DEBUG_SERIAL.println(motPos[2]);
  MoveMotorToGoal(1, 50, motPos[0]);
  MoveMotorToGoal(2, 50, motPos[1]);
  MoveMotorToGoal(3, 50, motPos[2]);
  WaitMotorsTakeGoalPos(motPos[0], motPos[1], motPos[2]);*/
  delay(500);
  DEBUG_SERIAL.println();
}

int ConvertDegreesToGoalPos(float deg) {
  // 30° - мертвая зона диномикселя
  deg = constrain(deg, 30, 300); // Ограничиваем входное значение, где 30° - это начальный градус слева и 300°
  float goalPos = map(deg, 330, 30, 1023, 0);
  return goalPos;
}

// Ждать пока моторы не займут позиции

/*
void WaitMotorsTakeGoalPos(int *motorsPos) {
  while (true) {
    DEBUG_SERIAL.print("Motors position: ");
    DEBUG_SERIAL.print(dxl.getPresentPosition(1)); DEBUG_SERIAL.print(", ");
    DEBUG_SERIAL.print(dxl.getPresentPosition(2)); DEBUG_SERIAL.print(", ");
    DEBUG_SERIAL.print(dxl.getPresentPosition(3)); DEBUG_SERIAL.print(", ");
    DEBUG_SERIAL.print(dxl.getPresentPosition(4)); DEBUG_SERIAL.print(", ");
    DEBUG_SERIAL.print(dxl.getPresentPosition(5)); DEBUG_SERIAL.print(", ");
    DEBUG_SERIAL.println(dxl.getPresentPosition(6));
    if ((motorsPos[0] - DYNAMIXEL_GOAL_POS_ERROR <= dxl.getPresentPosition(1) && dxl.getPresentPosition(1) <= motorsPos[0] + DYNAMIXEL_GOAL_POS_ERROR) && (motorsPos[1] - DYNAMIXEL_GOAL_POS_ERROR <= dxl.getPresentPosition(2) && dxl.getPresentPosition(2) <= motorsPos[1] + DYNAMIXEL_GOAL_POS_ERROR) && (motorsPos[2] - DYNAMIXEL_GOAL_POS_ERROR <= dxl.getPresentPosition(3) && dxl.getPresentPosition(3) <= motorsPos[2] + DYNAMIXEL_GOAL_POS_ERROR) && (motorsPos[3] - DYNAMIXEL_GOAL_POS_ERROR <= dxl.getPresentPosition(4) && dxl.getPresentPosition(4) <= motorsPos[3] + DYNAMIXEL_GOAL_POS_ERROR) && (motorsPos[4] - DYNAMIXEL_GOAL_POS_ERROR <= dxl.getPresentPosition(5) && dxl.getPresentPosition(5) <= motorsPos[4] + DYNAMIXEL_GOAL_POS_ERROR)) {
      break;
    }
    //delay(500);
  }
  DEBUG_SERIAL.print("Motors performed position: ");
  DEBUG_SERIAL.print(dxl.getPresentPosition(1)); DEBUG_SERIAL.print(", ");
  DEBUG_SERIAL.print(dxl.getPresentPosition(2)); DEBUG_SERIAL.print(", ");
  DEBUG_SERIAL.print(dxl.getPresentPosition(3)); DEBUG_SERIAL.print(", ");
  DEBUG_SERIAL.print(dxl.getPresentPosition(4)); DEBUG_SERIAL.print(", ");
  DEBUG_SERIAL.print(dxl.getPresentPosition(5)); DEBUG_SERIAL.print(", ");
  DEBUG_SERIAL.println(dxl.getPresentPosition(6));
}
*/

void WaitMotorsTakeGoalPos() {
  int motorsGoalPos[JOINT_N];
  for (byte i = 0; i <= JOINT_N; i++) {
    motorsGoalPos[i] = dxl.readControlTableItem(PRESENT_POSITION, i + 1);
  }
  while (true) {
    DEBUG_SERIAL.print("Motors position: ");
    DEBUG_SERIAL.print(dxl.getPresentPosition(1)); DEBUG_SERIAL.print(", ");
    DEBUG_SERIAL.print(dxl.getPresentPosition(2)); DEBUG_SERIAL.print(", ");
    DEBUG_SERIAL.print(dxl.getPresentPosition(3)); DEBUG_SERIAL.print(", ");
    DEBUG_SERIAL.print(dxl.getPresentPosition(4)); DEBUG_SERIAL.print(", ");
    DEBUG_SERIAL.print(dxl.getPresentPosition(5)); DEBUG_SERIAL.print(", ");
    DEBUG_SERIAL.println(dxl.getPresentPosition(6));
    if ((motorsGoalPos[0] - DYNAMIXEL_GOAL_POS_ERROR <= dxl.getPresentPosition(1) && dxl.getPresentPosition(1) <= motorsGoalPos[0] + DYNAMIXEL_GOAL_POS_ERROR) && (motorsGoalPos[1] - DYNAMIXEL_GOAL_POS_ERROR <= dxl.getPresentPosition(2) && dxl.getPresentPosition(2) <= motorsGoalPos[1] + DYNAMIXEL_GOAL_POS_ERROR) && (motorsGoalPos[2] - DYNAMIXEL_GOAL_POS_ERROR <= dxl.getPresentPosition(3) && dxl.getPresentPosition(3) <= motorsGoalPos[2] + DYNAMIXEL_GOAL_POS_ERROR) && (motorsGoalPos[3] - DYNAMIXEL_GOAL_POS_ERROR <= dxl.getPresentPosition(4) && dxl.getPresentPosition(4) <= motorsGoalPos[3] + DYNAMIXEL_GOAL_POS_ERROR) && (motorsGoalPos[4] - DYNAMIXEL_GOAL_POS_ERROR <= dxl.getPresentPosition(5) && dxl.getPresentPosition(5) <= motorsGoalPos[4] + DYNAMIXEL_GOAL_POS_ERROR)) {
      break;
    }
    //delay(500);
  }
  DEBUG_SERIAL.print("Motors performed position: ");
  DEBUG_SERIAL.print(dxl.getPresentPosition(1)); DEBUG_SERIAL.print(", ");
  DEBUG_SERIAL.print(dxl.getPresentPosition(2)); DEBUG_SERIAL.print(", ");
  DEBUG_SERIAL.print(dxl.getPresentPosition(3)); DEBUG_SERIAL.print(", ");
  DEBUG_SERIAL.print(dxl.getPresentPosition(4)); DEBUG_SERIAL.print(", ");
  DEBUG_SERIAL.print(dxl.getPresentPosition(5)); DEBUG_SERIAL.print(", ");
  DEBUG_SERIAL.println(dxl.getPresentPosition(6));
}

// Задать позицию для диномикселя
void MoveMotorToGoal(byte motorId, byte speed, byte goalPos) {
  dxl.setGoalVelocity(motorId, speed); // Задание целевой скорости
  dxl.setGoalPosition(motorId, goalPos); // Задание целевого положения
  //dxl.setGoalPosition(1, 512); //use default encoder value
  //dxl.setGoalPosition(2, 45.0, UNIT_DEGREE); // Use angle in degree
}

// Функция обратной кинематики
float* Manipulator_IK(float x, float y, float z, float X_V, float Y_V, float Z_V) {
  float a1 = atan(y / x);
  float k = sqrt(pow(x, 2) + pow(y, 2));
  float z_solve = z + LINK4 - LINK1;
  float d = sqrt(pow(k, 2) + pow(z_solve, 2));
  float a2 = (3.14 / 2) - (atan(z_solve / k) + acos((d * d) + (LINK2 * LINK2) - (LINK3 * LINK3)) / (2 *  d * LINK2));
  float a3 = 3.14 - acos(((-d * -d) + (LINK2 * LINK2) - (LINK3 * LINK3)) / (2 * LINK2 * LINK3));
  float a4 = 3.14 - (a2 - a3);
  float a5 = a1;
  float *ik = new float[5];
  ik[0] = a1, ik[1] = a2, ik[2] = a3, ik[3] = a4, ik[4] = a5;
  return ik;
}

// Функция прямой кинематики
float* Manipulator_FK(float a1, float a2, float a3, float a4) {
  float x_a = LINK1 * cos(a1);
  float y_a = LINK1 * sin(a1);
  float x_ba = LINK2 * cos(a1 + a2);
  float y_ba = LINK2 * sin(a1 + a2);
  float x_cba = LINK3 * cos(a1 + a2 + a3);
  float y_cba = LINK3 * sin(a1 + a2 + a3);
  float x_dcba = LINK4 * cos(a1 + a2 + a3 + a4);
  float y_dcba = LINK4 * sin(a1 + a2 + a3 + a4);
  float x = x_a + x_ba + x_cba + x_dcba;
  float y = y_a + y_ba + y_cba + y_dcba;
  float *fk = new float[2];
  fk[0] = x, fk[1] = y;
  return fk;
}

// Получить значения углов с моторов
int* GetMotorsGoalPos() {
  int *goalsPos = new int[JOINT_N];
  for (int i = 0; i < JOINT_N; i++) {
    goalsPos[i] = dxl.getPresentPosition(i + 1);
  }
  return goalsPos;
}
