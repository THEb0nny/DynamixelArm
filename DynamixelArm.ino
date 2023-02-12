// https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/
// https://emanual.robotis.com/docs/en/parts/controller/opencm904/
// https://bestprogrammer.ru/programmirovanie-i-razrabotka/vozvrat-massiva-iz-funktsii-c
// https://ru.stackoverflow.com/questions/526433/

#include <Dynamixel2Arduino.h>  // Подключение библиотеки Dynamixel
#include <math.h>
#include "GyverTimer.h"

#define DEBUG_LEVEL 2 // Уровень отдалки

#define MAX_INPUT_VAL_IN_MANUAL_CONTROL 6 // Максимальное количество значений в строку монитора порта при ручном управлении

#define DEBUG_SERIAL Serial // Установка константы, отвечающей за последовательный порт, подключаемый к компьютеру
#define DXL_SERIAL Serial3 // OpenCM9.04 EXP Board's DXL port Serial. (To use the DXL port on the OpenCM 9.04 board, you must use Serial1 for Serial. And because of the OpenCM 9.04 driver code, you must call Serial1.setDxlMode(true); before dxl.begin();.)

#define DXL_DIR_PIN 22 // Инициализация переменной, отвечащей за номер пина, подключенного к информационному пину приводов манипулятора
#define DXL_PROTOCOL_VERSION 1.0 // Инициализация переменной, отвечащей за протокол передачи данных от OpenCM9.04 к приводам

#define JOINT_N 6 // Количество приводов
#define DYNAMIXEL_DEG_OFFSET 30 // Оступ начала 0 позиции в градусах от разворота диномикселя
#define DYNAMIXEL_GOAL_POS_ERROR 3 // Погрешность позиции для динимикселей
#define MAX_TIME_PERFORMED_POS 7000 // Максимальное время для занятия ошибка, защита

#define EXP_BOARD_BUTTON1_PIN 16 // Пин кнопки 1 на плате расширения
#define EXP_BOARD_BUTTON2_PIN 17 // Пин кнопки 2 на плате расширения
#define EXP_BOARD_LED1_PIN 18 // Пин светодиода 1 на плате расширения
#define EXP_BOARD_LED2_PIN 19 // Пин светодиода 2 на плате расширения
#define EXP_BOARD_LED3_PIN 20 // Пин светодиода 3 на плате расширения

// Длины звеньев манипуля
#define LINK1 102.5
#define LINK2 105.0
#define LINK3 79.5
#define LINK4 164.5

// Для светодиодов на плате расширения, которые от земли
#define LED_HIGH LOW
#define LED_LOW HIGH

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN); // Инициализация указателя на команды из библиотеки Dynamixel
GTimer servosWorksMaxTimeTimer(MS); // Инициализация таймера защиты максимального времени цикла ожидания занятия позиции сервопривода
GTimer serialPrintTimer(MS);

using namespace ControlTableItem; // Пространство имён для обращения к параметрам диномикселей

byte robotState = 0; // Режим управления

int x = 0, y = 0, z = 0, tool;
int oldX = x, oldY = y, oldZ = z, oldTool = tool;

double* servosPos = new double[JOINT_N];
double* servosPosPrev = new double[JOINT_N];

String inputValues[MAX_INPUT_VAL_IN_MANUAL_CONTROL]; // Массив входящей строки
String key[MAX_INPUT_VAL_IN_MANUAL_CONTROL]; // Массив ключей
int values[MAX_INPUT_VAL_IN_MANUAL_CONTROL]; // Массив значений

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
  DEBUG_SERIAL.println("Wait press btn1 (auto mode) or btn2 (manual control)...");
  while(robotState == 0) { // Цикл ожидания нажатия кнопки
    if (digitalRead(EXP_BOARD_BUTTON1_PIN) == 1) { // Режим управления по x y z с решением ИК, кнопка 1 на плате расширения - 16
      robotState = 1;
    } else if (digitalRead(EXP_BOARD_BUTTON2_PIN) == 1) { // Режим управления диномикселями, кнопка 2 на плате расширения - 17
      robotState = 2;
    }
  }
  DEBUG_SERIAL.println("Setup...");
  dxl.begin(1000000); // Установка скорости обмена данными по последовательному порту манипулятора
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION); // Выбор протокола обмена данными
  serialPrintTimer.setInterval(500); // Установить таймер печати
  serialPrintTimer.reset(); // Спросить таймер печати
  for (int i = 0; i < JOINT_N; i++) { // Цикл для перебора всех приводов
    while (true) {
      if(dxl.ping(i + 1) == true) { // Проверка отвечает ли мотор
        DEBUG_SERIAL.println("Dynamixel width ID " + String(i + 1) + " found, model " + String(dxl.getModelNumber(i + 1)) + ".");
        //dxl.setBaudrate(i + 1, 1000000); // Установка битрейта, установится если только битрейт dxl.begin был такой же, что и на моторах
        break;
      } else if (serialPrintTimer.isReady()) {
        DEBUG_SERIAL.println("Dynamixel width ID " + String(i + 1) + " not found! Wait...");
      }
    }
    while (true) { // Установить режим работы
      dxl.torqueOff(i + 1); // Отключение крутящего момента, чтобы установить режим работы!
      bool setDinamixelOperationMode = dxl.setOperatingMode(i + 1, OP_POSITION); // Установка режима работы привода в качестве шарнира
      if (!setDinamixelOperationMode && serialPrintTimer.isReady()) {
        DEBUG_SERIAL.println("Dynamixel width ID" + String(i + 1) + " mode not set!");
      } else break;
    }
    dxl.torqueOn(i + 1); // Включение крутящего момента
  }
  
  DEBUG_SERIAL.println("Start... State is " + String(robotState));
  SetAllServosSpeed(60); // Установить всем сервоприводам скорость
  // Занять среднюю позицию всем сервоприводам
  double presentServosPos[] = {512, 512, 512, 512, 512, -1};
  MoveServosToGoalPos(presentServosPos, true);
  for (byte i = 0; i < JOINT_N; i++) { // Изначально задать в массиве средние значения для позий диномикселей
    servosPos[i] = 512;
    servosPosPrev[i] = 512;
  }
}

void loop() {
  if (robotState == 1) {
    ManualControl(1);
  } else {
    ManualControl(2);
  }
}

// Управление из Serial
void ManualControl(byte type) {
  if (Serial.available() > 2) { // Если есть доступные данные с Serial
    // Встроенная функция readStringUntil будет читать все данные, пришедшие в UART до специального символа — '\n' (перенос строки).
    // Он появляется в паре с '\r' (возврат каретки) при передаче данных функцией Serial.println().
    // Эти символы удобно передавать для разделения команд, но не очень удобно обрабатывать. Удаляем их функцией trim().
    String inputStr = Serial.readStringUntil('\n');
    inputStr.trim(); // Чистим символы
    char strBuffer[99]; // Создаём пустой массив символов
    inputStr.toCharArray(strBuffer, 99); // Перевести строку в массив символов последующего разделения по пробелам
    // Считываем x и y разделённых пробелом, а также Z и инструментом
    for (byte i = 0; i < MAX_INPUT_VAL_IN_MANUAL_CONTROL; i++) {
      inputValues[i] = (i == 0 ? String(strtok(strBuffer, " ")) : String(strtok(NULL, " ")));
      inputValues[i].replace(" ", ""); // Убрать возможные пробелы между символами
      /*
      if (DEBUG_LEVEL >= 2) {
        if (inputValues[i] != "") {
          if (i > 0) Serial.print(", ");
          Serial.print(inputValues[i]);
        }
        if (i == MAX_INPUT_VAL_IN_MANUAL_CONTROL - 1) Serial.println();
      }
      */
    }
    for (byte i = 0; i < MAX_INPUT_VAL_IN_MANUAL_CONTROL; i++) {
      if (inputValues[i] == "") continue; // Если значение пустое, то перейти на следующий шаг цикла
      String inputValue = inputValues[i]; // Записываем в строку обрезанную часть пробелами
      byte separatorIndexTmp = inputValue.indexOf("=");
      byte separatorIndex = (separatorIndexTmp != 255 ? separatorIndexTmp : inputValue.length());
      key[i] = inputValue.substring(0, separatorIndex); // Записываем ключ с начала строки до знака равно
      values[i] = (inputValue.substring(separatorIndex + 1, inputValue.length())).toInt(); // Записываем значение с начала цифры до конца строки
      if (type == 1) {
        if (key[i] == "x") {
          if (x != values[i]) x = values[i]; // Записываем X
        } else if (key[i] == "y") {
          if (y != values[i]) y = values[i]; // Записываем Y
        } else if (key[i] == "z") {
          if (z != values[i]) z = values[i]; // Записываем Z
        } else if (key[i] == "break") { // Выходим из цикла
          Serial.println(key[i]);
          robotState = 0;
          break;
        }
      } else if (type == 2) {
        if (key[i] == "m1") { // Значение первого диномикселя в градусах, котоое переконвертировано в позицию
          if (servosPos[0] != values[i]) servosPos[0] = ConvertDegreesToGoalPos(values[i]);
        } else if (key[i] == "m1r") { // Сырое значение первого диномикселя, т.е. goal position
          if (servosPos[0] != values[i]) servosPos[0] = values[i];
        } else if (key[i] == "m2") {
          if (servosPos[1] != values[i]) servosPos[1] = ConvertDegreesToGoalPos(values[i]);
        } else if (key[i] == "m2r") {
          if (servosPos[1] != values[i]) servosPos[1] = values[i];
        } else if (key[i] == "m3") {
          if (servosPos[2] != values[i]) servosPos[2] = ConvertDegreesToGoalPos(values[i]);
        } else if (key[i] == "m3r") {
          if (servosPos[2] != values[i]) servosPos[2] = values[i];
        } else if (key[i] == "m4") {
          if (servosPos[3] != values[i]) servosPos[3] = ConvertDegreesToGoalPos(values[i]);
        } else if (key[i] == "m4r") {
          if (servosPos[3] != values[i]) servosPos[3] = values[i];
        } else if (key[i] == "m5") {
          if (servosPos[4] != values[i]) servosPos[4] = ConvertDegreesToGoalPos(values[i]);
        } else if (key[i] == "m5r") {
          if (servosPos[4] != values[i]) servosPos[4] = values[i];
        } else if (key[i] == "m6") {
          if (servosPos[5] != values[i]) servosPos[5] = ConvertDegreesToGoalPos(values[i]);
        } else if (key[i] == "m6r") {
          if (servosPos[5] != values[i]) servosPos[5] = values[i];
        } else if (key[i] == "break") { // Выходим из цикла
          Serial.println(key[i]);
          robotState = 0;
          break;
        }
      }
      if (key[i].length() > 0) {
        DEBUG_SERIAL.println(String(key[i]) + " = " + String(values[i])); // Печать ключ и значение, если ключ существует
      }
    }
    // Тип работы по координатам X, Y, Z
    if (type == 1) {
      servosPos = Manipulator_IK(x, y, z); // Расчитать значения углов сервоприводов функцией Обратной кинематики
      DEBUG_SERIAL.println();
      for (byte i = 0; i < JOINT_N; i++) { // Перевести и переконвертировать значение Goal Position
        servosPos[i] = ConvertDegreesToGoalPos(90 + servosPos[i]); // Среднее положение 512
      }
      DEBUG_SERIAL.println();
      MoveServosToGoalPos(servosPos, true); // Занять диномикселям позицию и ждать выполнения
      for (byte i = 0; i < JOINT_N - 1; i++) { // Перезаписать значения старых позиций с текущей итерации для следующей итерации
        servosPosPrev[i] = servosPos[i];
      }
    } else if (type == 2) { // Тип работы по позициям на диномиксели
      if (servosPos[0] != servosPosPrev[0] || servosPos[1] != servosPosPrev[1] || servosPos[2] != servosPosPrev[2] || servosPos[3] != servosPosPrev[3] || servosPos[4] != servosPosPrev[4] || servosPos[5] != servosPosPrev[5] || servosPos[6] != servosPosPrev[6]) {
        MoveServosToGoalPos(servosPos, true); // Занять диномикселям позицию и ждать выполнения
        for (byte i = 0; i < JOINT_N; i++) { // Перезаписать значения старых позиций для следующей итерации
          servosPosPrev[i] = servosPos[i];
        }
      }
    }
  }
}

// Функция обратной кинематики
double* Manipulator_IK(float x, float y, float z) {
  double z_solve = z + LINK4 - LINK1;
  double k = sqrtf(pow(x, 2) + pow(y, 2));
  double d = sqrtf(pow(k, 2) + pow(z_solve, 2));
  double a1 = atan(x / y);
  double a2 = (PI / 2) - (atan(z_solve / k) + acos((pow(d, 2) + pow(LINK2, 2) - pow(LINK3, 2)) / (2 * d * LINK2)));
  double a3 = PI - (acos((-pow(d, 2) + pow(LINK2, 2) + pow(LINK3, 2)) / (2 * LINK2 * LINK3)));
  double a4 = PI - (a2 + a3);
  double a5 = a1;
  if (DEBUG_LEVEL >= 2) {
    DEBUG_SERIAL.println("IK:");
    DEBUG_SERIAL.println("k = " + String(k));
    DEBUG_SERIAL.println("z_solve = " + String(z_solve));
    DEBUG_SERIAL.println("d = " + String(d));
    DEBUG_SERIAL.println("a1_rad = " + String(a1));
    DEBUG_SERIAL.println("a2_rad = " + String(a2));
    DEBUG_SERIAL.println("a3_rad = " + String(a3));
    DEBUG_SERIAL.println("a4_rad = " + String(a4));
    DEBUG_SERIAL.println("a5_rad = " + String(a5));
    DEBUG_SERIAL.println("a1_deg = " + String(degrees(a1)));
    DEBUG_SERIAL.println("a2_deg = " + String(degrees(a2)));
    DEBUG_SERIAL.println("a3_deg = " + String(degrees(a3)));
    DEBUG_SERIAL.println("a4_deg = " + String(degrees(a4)));
    DEBUG_SERIAL.println("a5_deg = " + String(degrees(a5)));
  }
  double *ik = new double[JOINT_N - 1];
  ik[0] = degrees(a1), ik[1] = degrees(a2), ik[2] = degrees(a3), ik[3] = degrees(a4), ik[4] = degrees(a5);
  return ik;
}

// Функция прямой кинематики
double* Manipulator_FK(double a1, double a2, double a3, double a4) {
  double x_a = LINK1 * cos(a1);
  double y_a = LINK1 * sin(a1);
  double x_ba = LINK2 * cos(a1 + a2);
  double y_ba = LINK2 * sin(a1 + a2);
  double x_cba = LINK3 * cos(a1 + a2 + a3);
  double y_cba = LINK3 * sin(a1 + a2 + a3);
  double x_dcba = LINK4 * cos(a1 + a2 + a3 + a4);
  double y_dcba = LINK4 * sin(a1 + a2 + a3 + a4);
  double x = x_a + x_ba + x_cba + x_dcba;
  double y = y_a + y_ba + y_cba + y_dcba;
  double *fk = new double[3];
  fk[0] = x, fk[1] = y; // где z?
  return fk;
}

int ConvertDegreesToGoalPos(double degPos) {
  // .. > 30, 330 < .. - физически мертвые зоны диномикселя
  // Динамиксель команду 1023 - не выполняет, соотвественно 511 средняя позиция, а не как пишет документация 512
  if (DEBUG_LEVEL >= 1) {
    Serial.println("ConvertDegreesToGoalPos: ");
    Serial.print("inputDegPos: " + String(degPos) + ", ");
  }
  degPos = map(degPos, 0, 360, 360, 0); // Перевернуть диапазон вращения по особенностям диномикселя
  if (DEBUG_LEVEL >= 1) Serial.print("invertDegPos: " + String(degPos) + ", ");
  degPos -= DYNAMIXEL_DEG_OFFSET; // Отнять значение угла стартового положения
  degPos = constrain(degPos, 0, 300); // Ограничиваем входное значение, где 0° - это начальный градус слева и 300° конечный
  if (DEBUG_LEVEL >= 1) Serial.print("processedDegPos: " + String(degPos) + ", ");
  int goalPos = map(degPos, 0, 300, 0, 1023);
  if (DEBUG_LEVEL >= 1) Serial.println("goalPos: " + String(goalPos));
  return goalPos;
}

// Установить скорость сервоприводу
void SetServoSpeed(byte servoId, int speed) {
  bool status = dxl.setGoalVelocity(servoId, speed); // Задание целевой скорости
}

// Установить скорость сервоприводам
void SetServosSpeed(float *servosSpeed) {
  for (byte i = 0; i < JOINT_N; i++) {
    bool status = dxl.setGoalVelocity(i + 1, servosSpeed[i]); // Задание целевой скорости
  }
}

// Установить скорость всем сервоприводам
void SetAllServosSpeed(int speed) {
  for (byte i = 0; i < JOINT_N; i++) {
    bool status = dxl.setGoalVelocity(i + 1, speed); // Задание целевой скорости
  }
}

// Сервоприводу занять позицию
void MoveServoToPos(byte servoId, double posDeg) {
  bool status = dxl.setGoalPosition(servoId, posDeg); // Задание целевого положения
}

// Сервоприводам занять позиции
void MoveServosToGoalPos(double *servosTargetPos, bool waitPerformedPos) {
  if (DEBUG_LEVEL >= 1) DEBUG_SERIAL.print("Target servos pos: ");
  for (byte i = 0; i < JOINT_N; i++) {
    if (servosTargetPos[i] != -1 && servosTargetPos[i] < 1023) { // Пропустить шаг цикла, если значение для него с массива 0-е
      bool status = dxl.setGoalPosition(i + 1, servosTargetPos[i]); // Задание целевого положения
      if (DEBUG_LEVEL >= 1) DEBUG_SERIAL.print(servosTargetPos[i]);
    } else DEBUG_SERIAL.print("null");
    if (DEBUG_LEVEL >= 1) {
      if (i < JOINT_N - 1) DEBUG_SERIAL.print(", ");
      else DEBUG_SERIAL.println();
    }
  }
  if (waitPerformedPos) WaitServosPosPerformed(); // Если нужно, ждать занятия сервоприводами позиции
}

// Получить от серво его угол
int GetServoPos(byte servoId) {
  return dxl.getPresentPosition(servoId);
}

// Получить значения углов с сервоприводов
int* GetServosPos() {
  int* pos = new int[JOINT_N];
  for (byte i = 0; i < JOINT_N; i++) {
    pos[i] = dxl.getPresentPosition(i + 1);
  }
  return pos;
}

// Получить значения о движения сервопривода
bool GetServoMovingStatus(byte servoId) {
  return dxl.readControlTableItem(MOVING, servoId);
}

// Получить значения о движении сервоприводов
bool* GetServosMovingStatus() {
  bool* movingStates = new bool[JOINT_N];
  for (byte i = 0; i < JOINT_N; i++) {
    movingStates[i] = dxl.readControlTableItem(MOVING, i + 1);
  }
  return movingStates;
}

// Получить целевое значение с сервопривода
int GetServoTargetPos(byte servoId) {
  return dxl.readControlTableItem(GOAL_POSITION, servoId);
}

// Получить целевые значения с сервоприводов
int* GetServosTargetPos() {
  int* targetPos = new int[JOINT_N];
  for (byte i = 0; i < JOINT_N; i++) {
    targetPos[i] = dxl.readControlTableItem(GOAL_POSITION, i + 1);
  }
  return targetPos;
}

// Ждать пока сервомоторы не займут позиции
void WaitServosPosPerformed() {
  int* presentServosPos = new int[JOINT_N]; // Массив значений 
  bool* isMoving = new bool[JOINT_N]; // Массив для хранения значений о состоянии движения диномикселей
  //int* targetServosPos = GetServosTargetPos(); // Получить целевые позиции с сервоприводов
  servosWorksMaxTimeTimer.setTimeout(MAX_TIME_PERFORMED_POS); // Установка времени таймера защиты по максимальному времени, запуск таймера
  servosWorksMaxTimeTimer.reset(); // Спросить таймера защиты
  serialPrintTimer.setInterval(200); // Установить таймер печати, т.е. каждые n секунд будет печать
  serialPrintTimer.reset(); // Спросить таймер печати
  if (DEBUG_LEVEL >= 1) DEBUG_SERIAL.println("Current servos position: ");
  while (true) {
    //int* presentServosPos = GetServosPos(); // Прочитать значения с диномикселей
    presentServosPos = GetServosPos(); // Прочитать значения с диномикселей
    if (DEBUG_LEVEL >= 1 && serialPrintTimer.isReady()) {
      for (byte i = 0; i < JOINT_N; i++) { // Вывод текущих положений с сервоприводов
        DEBUG_SERIAL.print(presentServosPos[i]);
        if (i < JOINT_N - 1) DEBUG_SERIAL.print(", ");
        else DEBUG_SERIAL.println();
      }
    }
    //bool* isMoving = GetServosMovingStatus(); // Прочитать значения о состоянии движений
    isMoving = GetServosMovingStatus(); // Прочитать значения о состоянии движений
    if (DEBUG_LEVEL >= 2 && serialPrintTimer.isReady()) {
      for (byte i = 0; i < JOINT_N; i++) { // Вывод значений о движении сервоприводов
        DEBUG_SERIAL.print(isMoving[i]); // 1 - движение, 0 - движения нет
        if (i < JOINT_N - 1) DEBUG_SERIAL.print(", ");
        else DEBUG_SERIAL.println();
      }
    }
    /*
    // Проверяем, допустима ли ошибка
    bool* servosIsPerformed = new bool[JOINT_N]; // Массив состояния о занятии позиции сервоприводами
    for (byte i = 0; i < JOINT_N; i++) { // Проверяем условие и записываем в массив для каждого отдельного серво
      servosIsPerformed[i] = (targetServosPos[i] - DYNAMIXEL_GOAL_POS_ERROR <= presentServosPos[i] && presentServosPos[i] <= targetServosPos[i] + DYNAMIXEL_GOAL_POS_ERROR);
    }
    */
    // Если все условия выполнились по серво или превышено максимальное время по таймеру, то выйти из цикла
    // (servosIsPerformed[0] && servosIsPerformed[1] && servosIsPerformed[2]) && 
    if ((isMoving[0] == 0 && isMoving[1] == 0 && isMoving[2] == 0 && isMoving[3] == 0 && isMoving[4] == 0 && isMoving[5] == 0) || servosWorksMaxTimeTimer.isReady()) {
      if (DEBUG_LEVEL >= 1) {
        DEBUG_SERIAL.print("Motors performed position: ");
        for (byte i = 0; i < JOINT_N; i++) {
          DEBUG_SERIAL.print(presentServosPos[i]);
          if (i < JOINT_N - 1) DEBUG_SERIAL.print(", ");
          else DEBUG_SERIAL.println();
        }
      }
      // Удалить массивы перед выходом из цикла для очистки памяти
      //delete[] presentServosPos;
      //delete[] isMoving;
      //delete[] targetServosPos;
      //delete[] servosIsPerformed;
      break;
    }
    // Удаляем массивы для очистки памяти
    //delete[] presentServosPos;
    //delete[] isMoving;
    //delete[] targetServosPos;
    //delete[] servosIsPerformed;
    delay(50);
  }
  // Удалить массивы перед выходом из цикла для очистки памяти
  delete[] presentServosPos;
  delete[] isMoving;
  serialPrintTimer.stop(); // Остановить таймер печати
}
