// https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/
// https://emanual.robotis.com/docs/en/parts/controller/opencm904/

#include <Dynamixel2Arduino.h>  // Подключение библиотеки Dynamixel
#include <math.h>
#include "GyverTimer.h"

#define DEBUG_LEVEL 2 // Уровень дебага

#define MAX_INPUT_VAL_IN_MANUAL_CONTROL 4 // Максимальное количество значений в строку монитора порта при ручном управлении

#define DEBUG_SERIAL Serial // Установка константы, отвечающей за последовательный порт, подключаемый к компьютеру
#define DXL_SERIAL Serial3 // OpenCM9.04 EXP Board's DXL port Serial. (To use the DXL port on the OpenCM 9.04 board, you must use Serial1 for Serial. And because of the OpenCM 9.04 driver code, you must call Serial1.setDxlMode(true); before dxl.begin();.)

#define DXL_DIR_PIN 22 // Инициализация переменной, отвечащей за номер пина, подключенного к информационному пину приводов манипулятора
#define DXL_PROTOCOL_VERSION 1.0 // Инициализация переменной, отвечащей за протокол передачи данных от OpenCM9.04 к приводам
#define JOINT_N 6 // Количество приводов
#define DYNAMIXEL_GOAL_POS_ERROR 3 // Погрешность позиции для динимикселей
#define MAX_TIME_PERFORMED_POS 7000 // Максимальное время для занятия ошибка, защита

#define EXP_BOARD_BUTTON1_PIN 16 // Пин кнопки 1 на плате расширения
#define EXP_BOARD_BUTTON2_PIN 17 // Пин кнопки 2 на плате расширения
#define EXP_BOARD_LED1_PIN 18 // Пин светодиода 1 на плате расширения
#define EXP_BOARD_LED2_PIN 19 // Пин светодиода 2 на плате расширения
#define EXP_BOARD_LED3_PIN 20 // Пин светодиода 3 на плате расширения

// Длины звеньев манипуля
#define LINK1 103
#define LINK2 105
#define LINK3 89
#define LINK4 168

// Для светодиодов на плате расширения, которые от земли
#define LED_HIGH LOW
#define LED_LOW HIGH

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN); // Инициализация указателя на команды из библиотеки Dynamixel
GTimer servosWorksMaxTimeTimer(MS); // Инициализация таймера защиты максимального времени цикла ожидания занятия позиции сервопривода
GTimer serialPrintTimer(MS);

using namespace ControlTableItem;

byte workMode = 1; // Режим управления

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
  DEBUG_SERIAL.println("Wait press btn1 or btn2...");
  while(true) {
    if (digitalRead(EXP_BOARD_BUTTON1_PIN) == 1) { // Автоматический режим демонтрации
      workMode = 1;
      break; // Кнопка 1 на плате расширения
    }
    if (digitalRead(EXP_BOARD_BUTTON2_PIN) == 1) { // Режим управления
      workMode = 2;
      break; // Кнопка 2 на плате расширения
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
        DEBUG_SERIAL.print("Dynamixel width ID "); DEBUG_SERIAL.print(i + 1); DEBUG_SERIAL.print(" found, model "); DEBUG_SERIAL.print(dxl.getModelNumber(i + 1)); DEBUG_SERIAL.println(".");
        //dxl.setBaudrate(i + 1, 1000000); // Установка битрейта, установится если только битрейт dxl.begin был такой же, что и на моторах
        break;
      } else {
        if (serialPrintTimer.isReady()) {
          DEBUG_SERIAL.print("Dynamixel width ID "); DEBUG_SERIAL.print(i + 1); DEBUG_SERIAL.print(" not found!"); DEBUG_SERIAL.println(" Wait...");
        }
      }
    }
    while (true) { // Установить режим работы
      dxl.torqueOff(i + 1); // Отключение крутящего момента, чтобы установить режим работы!
      bool setDinamixelOperationMode = dxl.setOperatingMode(i + 1, OP_POSITION); // Установка режима работы привода в качестве шарнира
      if (!setDinamixelOperationMode && serialPrintTimer.isReady()) {
        DEBUG_SERIAL.print("Dynamixel width ID "); DEBUG_SERIAL.print(i + 1); DEBUG_SERIAL.println(" mode not set!");
      } else break;
    }
    dxl.torqueOn(i + 1); // Включение крутящего момента
  }
  DEBUG_SERIAL.print("Start... Work mode is "); DEBUG_SERIAL.println(workMode);
  SetAllServosSpeed(60); // Установить всем сервоприводам скорость
  // Занять среднюю позицию всем сервоприводам
  int presentServosPos[] = {512, 512, 512, 512, 512, 0};
  MoveServosToPos(presentServosPos, true);
}

void loop() {
  ManualControl(1);
  while(true);
}

int ConvertDegreesToGoalPos(float degPos) {
  // 30, 300 - мертвые зоны диномикселя
  degPos = constrain(degPos, 30, 300); // Ограничиваем входное значение, где 30° - это начальный градус слева и 300°
  int goalPos = map(degPos, 300, 30, 1023, 0);
  return goalPos;
}

// Ждать пока сервомоторы не займут позиции
void WaitServosPosPerformed() {
  int* presentServosPos = new int[JOINT_N]; // Массив для текущих значений на сервоприводах
  int* targetServosPos = GetServosTargetPos(); // Получить целевые позиции с сервоприводов
  bool* isMoving = GetServosMoving(); // Массив значений о движении сервоприводов
  bool* servosIsPerformed = new bool[JOINT_N]; // Массив состояния о занятии позиции сервоприводами
  servosWorksMaxTimeTimer.setTimeout(MAX_TIME_PERFORMED_POS); // Установка времени таймера защиты по максимальному времени, запуск таймера
  servosWorksMaxTimeTimer.reset(); // Спросить таймера защиты
  serialPrintTimer.setInterval(500); // Установить таймер печати
  serialPrintTimer.reset(); // Спросить таймер печати
  if (DEBUG_LEVEL >= 1) DEBUG_SERIAL.println("Current servos position: ");
  while (true) {
    presentServosPos = GetServosPos(); // Прочитать значения с диномикселей
    if (DEBUG_LEVEL >= 1 && serialPrintTimer.isReady()) {
      for (byte i = 0; i < JOINT_N; i++) { // Вывод текущих положений с сервоприводов
        DEBUG_SERIAL.print(presentServosPos[i]);
        if (i < JOINT_N - 1) DEBUG_SERIAL.print(", ");
        else DEBUG_SERIAL.println();
      }
    }
    isMoving = GetServosMoving(); // Прочитать значения о состоянии движений
    if (DEBUG_LEVEL >= 2 && serialPrintTimer.isReady()) {
      for (byte i = 0; i < JOINT_N; i++) { // Вывод значений о движении сервоприводов
        DEBUG_SERIAL.print(isMoving[i]); // 1 - движение, 0 - движения нет
        if (i < JOINT_N - 1) DEBUG_SERIAL.print(", ");
        else DEBUG_SERIAL.println();
      }
    }
    /*
    // Проверяем, допустима ли ошибка
    for (byte i = 0; i < JOINT_N; i++) { // Проверяем условие и записываем в массив для каждого отдельного серво
      servosIsPerformed[i] = (targetServosPos[i] - DYNAMIXEL_GOAL_POS_ERROR <= presentServosPos[i] && presentServosPos[i] <= targetServosPos[i] + DYNAMIXEL_GOAL_POS_ERROR);
    }
    */
    // Если все условия выполнились по серво или превышено максимальное время по таймеру, то выйти из цикла
    // (servosIsPerformed[0] && servosIsPerformed[1] && servosIsPerformed[2]) && 
    if ((isMoving[0] == 0 && isMoving[1] == 0 && isMoving[2] == 0 && isMoving[3] == 0 && isMoving[4] == 0 && isMoving[5] == 0) || servosWorksMaxTimeTimer.isReady()) break;
    delay(100);
  }
  if (DEBUG_LEVEL >= 1) {
    DEBUG_SERIAL.print("Motors performed position: ");
    for (byte i = 0; i < JOINT_N; i++) {
      DEBUG_SERIAL.print(presentServosPos[i]);
      if (i < JOINT_N - 1) DEBUG_SERIAL.print(", ");
      else DEBUG_SERIAL.println();
    }
  }
  serialPrintTimer.stop(); // Остановить таймер печати
  DEBUG_SERIAL.flush();
}

// Установить скорость сервоприводу
void SetServoSpeed(byte servoId, int speed) {
  dxl.setGoalVelocity(servoId, speed); // Задание целевой скорости
}

// Установить скорость сервоприводам
void SetServosSpeed(float *servosSpeed) {
  for (byte i = 0; i < JOINT_N; i++) {
    dxl.setGoalVelocity(i + 1, servosSpeed[i]); // Задание целевой скорости
  }
}

// Установить скорость всем сервоприводам
void SetAllServosSpeed(int speed) {
  for (byte i = 0; i < JOINT_N; i++) {
    dxl.setGoalVelocity(i + 1, speed); // Задание целевой скорости
  }
}

// Сервоприводу занять позицию
void MoveServoToPos(byte servoId, float posDeg) {
  dxl.setGoalPosition(servoId, posDeg); // Задание целевого положения
}

// Сервоприводам занять позиции
void MoveServosToPos(int *servosTargetPos, bool waitPerformedPos) {
  if (DEBUG_LEVEL >= 1) DEBUG_SERIAL.print("Target servos pos: ");
  for (byte i = 0; i < JOINT_N; i++) {
    if (servosTargetPos[i] != 0 && servosTargetPos[i] < 1023) { // Пропустить шаг цикла, если значение для него с массива 0-е
      dxl.setGoalPosition(i + 1, servosTargetPos[i]); // Задание целевого положения
      DEBUG_SERIAL.print(servosTargetPos[i]);
    } else DEBUG_SERIAL.print("null");
    if (i < JOINT_N - 1) DEBUG_SERIAL.print(", ");
    else DEBUG_SERIAL.println();
  }
  if (waitPerformedPos) WaitServosPosPerformed(); // Если нужно, ждать занятия сервоприводами позиции
}

// Получить от серво его угол
int GetServoPos(byte servoId) {
  return dxl.getPresentPosition(servoId);
}

// Получить значения углов с сервоприводов
int* GetServosPos() {
  int *pos = new int[JOINT_N];
  for (byte i = 0; i < JOINT_N; i++) {
    pos[i] = dxl.getPresentPosition(i + 1);
  }
  return pos;
}

// Получить значения о движения сервопривода
bool GetServoMoving(byte servoId) {
  return dxl.readControlTableItem(MOVING, servoId);
}

// Получить значения о движении сервоприводов
bool* GetServosMoving() {
  bool *movingStates = new bool[JOINT_N];
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
  int *targetPos = new int[JOINT_N];
  for (byte i = 0; i < JOINT_N; i++) {
    targetPos[i] = dxl.readControlTableItem(GOAL_POSITION, i + 1);
  }
  return targetPos;
}

// Функция обратной кинематики
float* Manipulator_IK(float x, float y, float z) {
  float a1 = atan(y / x);
  //Serial.println(a1);
  float k = sqrt(pow(x, 2) + pow(y, 2));
  Serial.println(k);
  float z_solve = z + LINK4 - LINK1;
  Serial.println(z_solve);
  float d = sqrt(pow(k, 2) + pow(z_solve, 2));
  Serial.println(d);
  float a2 = (PI / 2) - (atan(z_solve / k) + acos((pow(d, 2) + pow(LINK2, 2) - pow(LINK3, 2)) / (2 *  d * LINK2)));
  Serial.println(a2);
  float a3 = PI - acos((pow(-d, 2) + pow(LINK2, 2) + pow(LINK3, 2)) / (2 * LINK2 * LINK3));
  Serial.println(a3);
  float a4 = PI - (a2 + a3);
  Serial.println(a4);
  float a5 = a1;
  float *ik = new float[JOINT_N - 1];
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
  fk[0] = x, fk[1] = y; // где z?
  return fk;
}

// Управление из Serial
void ManualControl(int type) {
  int servosPos[] = {512, 512, 512, 512, 512, 512};
  int servosPosOld[] = {512, 512, 512, 512, 512, 512};
  float* servos = new float[JOINT_N];
  float* servosOld = new float[JOINT_N];
  int x = 0, y = 0, z = 0, tool;
  int oldX = x, oldY = y, oldZ = z, oldTool = tool;
  bool control = true;
  while (control) {
    String inputValues[MAX_INPUT_VAL_IN_MANUAL_CONTROL]; // Массив входящей строки
    String key[MAX_INPUT_VAL_IN_MANUAL_CONTROL]; // Массив ключей
    int values[MAX_INPUT_VAL_IN_MANUAL_CONTROL]; // Массив значений
    if (Serial.available() > 2) { // Если есть доступные данные
      // Встроенная функция readStringUntil будет читать все данные, пришедшие в UART до специального символа — '\n' (перенос строки).
      // Он появляется в паре с '\r' (возврат каретки) при передаче данных функцией Serial.println().
      // Эти символы удобно передавать для разделения команд, но не очень удобно обрабатывать. Удаляем их функцией trim().
      String inputStr = Serial.readStringUntil('\n');
      inputStr.trim(); // Чистим символы
      char strBuffer[99]; // Создаём пустой массив символов
      inputStr.toCharArray(strBuffer, 99); // Перевести строку в массив символов
      // Считываем x и y разделённых пробелом, а также Z и инструментом
      for (byte i = 0; i < MAX_INPUT_VAL_IN_MANUAL_CONTROL; i++) {
        inputValues[i] = (i == 0 ? String(strtok(strBuffer, " ")) : String(strtok(NULL, " ")));
        inputValues[i].replace(" ", ""); // Убрать возможные пробелы между символами
        if (DEBUG_LEVEL >= 2) {
          Serial.print(inputValues[i] != "" ? inputValues[i] : "");
          if (i < MAX_INPUT_VAL_IN_MANUAL_CONTROL - 1) Serial.print(", ");
          else Serial.println();
        }
      }
      for (byte i = 0; i < MAX_INPUT_VAL_IN_MANUAL_CONTROL; i++) {
        String inputValue = inputValues[i];
        byte strIndex = inputValue.length(); // Переменая для хронения индекса вхождения цифры в входной строке, изначально равна размеру строки
        for (byte i = 0; i < 10; i++) { // Поиск первого вхождения цифры от 0 по 9 в подстроку
          byte index = inputValue.indexOf(String(i)); // Узнаём индекс, где нашли цифру параметра цикла
          if (index < strIndex && index != 255) strIndex = index; // Если индекс цифры меньше strIndex, то обновляем strIndex 
        }
        key[i] = inputValue.substring(0, strIndex); // Записываем ключ с начала строки до первой цицры
        values[i] = (inputValue.substring(strIndex, inputValue.length())).toInt(); // Записываем значение с начала цифры до конца строки
        if (key[i] == "x" && type == 1) {
          if (x != values[i]) x = values[i]; // Записываем X
        } else if (key[i] == "y" && type == 1) {
          if (y != values[i]) y = values[i]; // Записываем Y
        } else if (key[i] == "z" && type == 1) {
          if (z != values[i]) z = values[i]; // Записываем Z
        } else if (key[i] == "one" && type == 2) {
          if (servosPos[0] != values[i]) servosPos[0] = values[i];
        } else if (key[i] == "two" && type == 2) {
          if (servosPos[1] != values[i]) servosPos[1] = values[i];
        } else if (key[i] == "three" && type == 2) {
          if (servosPos[2] != values[i]) servosPos[2] = values[i];
        } else if (key[i] == "four" && type == 2) {
          if (servosPos[3] != values[i]) servosPos[3] = values[i];
        } else if (key[i] == "five" && type == 2) {
          if (servosPos[4] != values[i]) servosPos[4] = values[i];
        } else if (key[i] == "six" && type == 2) {
          if (servosPos[5] != values[i]) servosPos[5] = values[i];
        } else if (key[i] == "break") {
          Serial.println(key[i]);
          control = false;
          break;
        }
        if (key[i].length() > 0) {
          Serial.print(key[i]); Serial.print(" = "); Serial.println(values[i]); // Печать ключ и значение, если ключ существует
        }
      }
      if (type == 1) { // Тип работы по координатам X и Y
        servos = Manipulator_IK(x, y, z);
        for(byte i = 0; i < JOINT_N - 1; i++) {
          Serial.print(servos[i]);
          if (i < JOINT_N - 1) Serial.print(", ");
          else Serial.println();
        }
        for (byte i = 0; i < JOINT_N - 1; i++) { // Перезаписать значения старых позиций для следующей итерации
            servosOld[i] = servos[i];
        }
      } else if (type == 2) { // Тип работы по позициям на диномиксели
        if (servosPos[0] != servosPosOld[0] || servosPos[1] != servosPosOld[1] || servosPos[2] != servosPosOld[2] || servosPos[3] != servosPosOld[3] || servosPos[4] != servosPosOld[4] || servosPos[5] != servosPosOld[5] || servosPos[6] != servosPosOld[6]) {
          MoveServosToPos(servosPos, true);
          for (byte i = 0; i < JOINT_N; i++) { // Перезаписать значения старых позиций для следующей итерации
            servosPosOld[i] = servosPos[i];
          }
        }
      }
    }
  }
}
