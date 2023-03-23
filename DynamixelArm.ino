// https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/
// https://emanual.robotis.com/docs/en/parts/controller/opencm904/
// https://bestprogrammer.ru/programmirovanie-i-razrabotka/vozvrat-massiva-iz-funktsii-c
// https://ru.stackoverflow.com/questions/526433/

#include <Dynamixel2Arduino.h>  // Подключение библиотеки Dynamixel
#include <math.h>
#include "GyverTimer.h"

using namespace ControlTableItem; // Пространство имён для обращения к параметрам диномикселей

#define CONVERT_DEG_TO_GOAL_POS_FUNC_DEBUG true // Отладка функции конвертирования градусов в Goal Position
#define MOVE_SERV_TO_GOAL_POS_FUNC_DEBUG true // Отладка функции управления перемещения до Goal Position
#define WAIT_SERV_POS_PERF_FUNC_DEBUG true // Отладка функции ожидания занятия позиций серваками
#define IK_FUNC_DEBUG true // Отладка функции обратной кинематики

#define MAX_INPUT_VAL_AT_MANUAL_CONTROL 6 // Максимальное количество значений в строку монитора порта при ручном управлении

#define DEBUG_SERIAL Serial // Установка константы, отвечающей за последовательный порт, подключаемый к компьютеру
#define DXL_SERIAL Serial3 // OpenCM9.04 EXP Board's DXL port Serial. (To use the DXL port on the OpenCM 9.04 board, you must use Serial1 for Serial. And because of the OpenCM 9.04 driver code, you must call Serial1.setDxlMode(true); before dxl.begin();.)

#define DXL_DIR_PIN 22 // Инициализация переменной, отвечащей за номер пина, подключенного к информационному пину приводов манипулятора
#define DXL_PROTOCOL_VERSION 1.0 // Инициализация переменной, отвечащей за протокол передачи данных от OpenCM9.04 к приводам

#define JOINT_N 6 // Количество сервоприводов
#define DYNAMIXEL_DEG_OFFSET 30 // Оступ начала 0 позиции в градусах от разворота диномикселя
#define MAX_TIME_PERFORMED_POS 7000 // Максимальное время в мм для занятии позиции, время для защиты от невозможности занять позицию 

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

byte robotState = 0; // Состояние робота
byte workType = 0; // Режим работы

int x, y, z, tool;

double* servosPos = new double[JOINT_N]{0}; // Массив для хранения значений о нужных позициях динамикселей
double* servosPosPrev = new double[JOINT_N]{0}; // Массив для хранения значений старых позиций динамикселей для сравнения перед выполнением

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
  while(workType == 0) { // Цикл ожидания нажатия кнопки
    if (digitalRead(EXP_BOARD_BUTTON1_PIN) == 1) { // Режим управления по x y z с решением ИК, кнопка 1 на плате расширения - 16
      workType = 1;
    } else if (digitalRead(EXP_BOARD_BUTTON2_PIN) == 1) { // Режим управления диномикселями, кнопка 2 на плате расширения - 17
      workType = 2;
    }
  }
  workType = 1;
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
  //SetAllServosSpeed(60); // Установить всем сервоприводам скорость
  
  double startServosPos[] = {512, 512, 512, 512, 512, -1}; // Создаём массив с значениями при старте
  MoveServosToGoalPos(startServosPos, true); // Занять среднюю позицию всем сервоприводам при старте
  // Установка значений массивов позиций, с которыми работает программа
  for(byte i = 0; i < JOINT_N; i++) {
    servosPos[i] = startServosPos[i];
    servosPosPrev[i] = servosPos[i];
  }
}

void loop() {
  if (workType == 1) { // Управление по x, y, z
    ManualControl(1);
  } else { // Управление диномикселями отдельно
    ManualControl(2);
  }
}

// Управление из Serial
void ManualControl(byte workType) {
  if (Serial.available() > 2) { // Если есть доступные данные с Serial
    String inputValues[MAX_INPUT_VAL_AT_MANUAL_CONTROL]; // Массив входящей строки
    String key[MAX_INPUT_VAL_AT_MANUAL_CONTROL]; // Массив ключей
    int values[MAX_INPUT_VAL_AT_MANUAL_CONTROL]; // Массив значений
    // Встроенная функция readStringUntil будет читать все данные, пришедшие в UART до специального символа — '\n' (перенос строки).
    // Он появляется в паре с '\r' (возврат каретки) при передаче данных функцией Serial.println().
    // Эти символы удобно передавать для разделения команд, но не очень удобно обрабатывать. Удаляем их функцией trim().
    String inputStr = Serial.readStringUntil('\n');
    inputStr.trim(); // Чистим символы
    char strBuffer[99]; // Создаём пустой массив символов
    inputStr.toCharArray(strBuffer, 99); // Перевести строку в массив символов последующего разделения по пробелам
    // Считываем x и y разделённых пробелом, а также z и инструмент
    for (byte i = 0; i < MAX_INPUT_VAL_AT_MANUAL_CONTROL; i++) {
      inputValues[i] = (i == 0 ? String(strtok(strBuffer, " ")) : String(strtok(NULL, " ")));
      inputValues[i].replace(" ", ""); // Убрать возможные пробелы между символами
    }

    DEBUG_SERIAL.println();
    for (byte i = 0; i < MAX_INPUT_VAL_AT_MANUAL_CONTROL; i++) {
      if (inputValues[i] == "") continue; // Если значение пустое, то перейти на следующий шаг цикла
      String inputValue = inputValues[i]; // Записываем в строку обрезанную часть пробелами
      byte separatorIndexTmp = inputValue.indexOf("="); // Узнаём позицию знака равно
      byte separatorIndex = (separatorIndexTmp != 255 ? separatorIndexTmp : inputValue.length());
      key[i] = inputValue.substring(0, separatorIndex); // Записываем ключ с начала строки до знака равно
      values[i] = (inputValue.substring(separatorIndex + 1, inputValue.length())).toInt(); // Записываем значение с начала цифры до конца строки
      
      if (workType == 1) { // Если режим работы 1
        robotState = 1; // Устанавливаем режим проверки перед работой
        if (key[i] == "x") x = values[i]; // Записываем X
        else if (key[i] == "y") y = values[i]; // Записываем Y
        else if (key[i] == "z") z = values[i]; // Записываем Z
        else if (key[i] == "break") { // Выходим из цикла
          robotState = 0; // Режим ожидания
          Serial.println(key[i]);
          break;
        } else { // Если нет подходящего ключа
          robotState = 0; // Режим ожидания, т.к. ключа верного не оказалось
        }
      } else if (workType == 2) { // Если режим работы 2
        robotState = 1; // Устанавливаем режим проверки перед работой
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
          robotState = 0; // Режим ожидания
          Serial.println(key[i]);
          break;
        } else { // Если нет подходящего ключа
          robotState = 0; // Режим ожидания, т.к. ключа верного не оказалось
        }
      }
      if (robotState == 1) robotState = 2; // Устанавливаем c режима проверки на режим работы
      if (key[i].length() > 0) { // Печать ключ и значение, если ключ существует
        DEBUG_SERIAL.println(String(key[i]) + " = " + String(values[i]));
      }
    }
    if (workType == 1 && robotState == 2) { // Если тип работы по координатам X, Y, Z, тогда решить IK и переконвертировать значения на углы и установлен режим работы
      double* ikServosDeg = Manipulator_IK(x, y, z); // Расчитать значения углов сервоприводов функцией обратной кинематики
      DEBUG_SERIAL.println();
      for (byte i = 0; i < JOINT_N; i++) { // Перевести и переконвертировать значение Goal Position
        servosPos[i] = ConvertDegreesToGoalPos(90 + ik_deg[i]); // Среднее положение 512
      }
      delete[] ikServosDeg; // Удалить память под массив из функции Manipulator_IK
    }
    // Проверка нужно ли выполнять, если значения углов пытаемся отдать теже самые
    if (robotState == 2 && servosPos[0] != servosPosPrev[0] || servosPos[1] != servosPosPrev[1] || servosPos[2] != servosPosPrev[2] || servosPos[3] != servosPosPrev[3] || servosPos[4] != servosPosPrev[4] || servosPos[5] != servosPosPrev[5]) {
      MoveServosToGoalPos(servosPos, true); // Занять диномикселям позицию и ждать выполнения
      robotState = 0; // Установить режим ожидания после того, как отработали
      for (byte i = 0; i < JOINT_N; i++) { // Перезаписать значения старых позиций с текущей итерации для следующей
        servosPosPrev[i] = servosPos[i];
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
  if (IK_FUNC_DEBUG) {
    DEBUG_SERIAL.println();
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
  double *ik = new double[JOINT_N - 1]{0};
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
  if (CONVERT_DEG_TO_GOAL_POS_FUNC_DEBUG) { // Выводить входные значения
    Serial.println("ConvertDegreesToGoalPos: ");
    Serial.print("inputDegPos: " + String(degPos) + ", ");
  }
  degPos = map(degPos, 0, 360, 360, 0); // Перевернуть диапазон вращения по особенностям диномикселя
  if (CONVERT_DEG_TO_GOAL_POS_FUNC_DEBUG) Serial.print("invertDegPos: " + String(degPos) + ", ");
  degPos -= DYNAMIXEL_DEG_OFFSET; // Отнять значение угла стартового положения
  degPos = constrain(degPos, 0, 300); // Ограничиваем входное значение, где 0° - это начальный градус слева и 300° конечный
  if (CONVERT_DEG_TO_GOAL_POS_FUNC_DEBUG) Serial.print("processedDegPos: " + String(degPos) + ", ");
  int goalPos = map(degPos, 0, 300, 0, 1023);
  if (CONVERT_DEG_TO_GOAL_POS_FUNC_DEBUG) Serial.println("goalPos: " + String(goalPos));
  return goalPos;
}

// Установить скорость сервоприводу по id
void SetServoSpeed(byte servoId, int speed) {
  dxl.torqueOff(servoId); // Работает?
  bool status = dxl.setGoalVelocity(servoId, speed); // Задание целевой скорости
  dxl.torqueOn(servoId);
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
  if (MOVE_SERV_TO_GOAL_POS_FUNC_DEBUG) {
    DEBUG_SERIAL.println();
    DEBUG_SERIAL.print("Target servos pos: ");
  }
  for (byte i = 0; i < JOINT_N; i++) {
    if (servosTargetPos[i] != -1 && servosTargetPos[i] < 1023) { // Пропустить шаг цикла, если значение для него с массива 0-е
      bool status = dxl.setGoalPosition(i + 1, servosTargetPos[i]); // Задание целевого положения
      if (MOVE_SERV_TO_GOAL_POS_FUNC_DEBUG) DEBUG_SERIAL.print(servosTargetPos[i]);
    } else DEBUG_SERIAL.print("null");
    if (MOVE_SERV_TO_GOAL_POS_FUNC_DEBUG) {
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
  int* presentServosPos = new int[JOINT_N]{0}; // Массив значений 
  bool* isMoving = new bool[JOINT_N]{0}; // Массив для хранения значений о состоянии движения диномикселей
  servosWorksMaxTimeTimer.setTimeout(MAX_TIME_PERFORMED_POS); // Установка времени таймера защиты по максимальному времени, запуск таймера
  servosWorksMaxTimeTimer.reset(); // Спросить таймера защиты
  if (WAIT_SERV_POS_PERF_FUNC_DEBUG) {
    serialPrintTimer.setInterval(200); // Установить таймер печати, т.е. каждые n секунд будет печать
    serialPrintTimer.reset(); // Спросить таймер печати
  }
  if (WAIT_SERV_POS_PERF_FUNC_DEBUG) DEBUG_SERIAL.println("Current servos position: ");
  while (true) {
    presentServosPos = GetServosPos(); // Прочитать значения с диномикселей
    if (WAIT_SERV_POS_PERF_FUNC_DEBUG && serialPrintTimer.isReady()) {
      for (byte i = 0; i < JOINT_N; i++) { // Вывод текущих положений с сервоприводов
        DEBUG_SERIAL.print(presentServosPos[i]);
        if (i < JOINT_N - 1) DEBUG_SERIAL.print(", ");
        else DEBUG_SERIAL.println();
      }
    }
    isMoving = GetServosMovingStatus(); // Прочитать значения о состоянии движений
    if (WAIT_SERV_POS_PERF_FUNC_DEBUG && serialPrintTimer.isReady()) {
      for (byte i = 0; i < JOINT_N; i++) { // Вывод значений о движении сервоприводов
        DEBUG_SERIAL.print(isMoving[i]); // 1 - движение, 0 - движения нет
        if (i < JOINT_N - 1) DEBUG_SERIAL.print(", ");
        else DEBUG_SERIAL.println();
      }
    }
    // Если все условия выполнились по серво или превышено максимальное время по таймеру, то выйти из цикла
    if ((isMoving[0] == 0 && isMoving[1] == 0 && isMoving[2] == 0 && isMoving[3] == 0 && isMoving[4] == 0 && isMoving[5] == 0) || servosWorksMaxTimeTimer.isReady()) {
      if (WAIT_SERV_POS_PERF_FUNC_DEBUG) {
        DEBUG_SERIAL.print("Motors performed position: ");
        for (byte i = 0; i < JOINT_N; i++) {
          DEBUG_SERIAL.print(presentServosPos[i]);
          if (i < JOINT_N - 1) DEBUG_SERIAL.print(", ");
          else DEBUG_SERIAL.println();
        }
      }
      break;
    }
    delay(10);
  }
  // Удалить массивы перед выходом из функции для очистки памяти
  delete[] presentServosPos;
  delete[] isMoving;
  if (WAIT_SERV_POS_PERF_FUNC_DEBUG) serialPrintTimer.stop(); // Остановить таймер печати
}
