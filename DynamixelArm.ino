// https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/
// https://emanual.robotis.com/docs/en/parts/controller/opencm904/

#include <Dynamixel2Arduino.h>  // Подключение библиотеки Dynamixel
#include <math.h>

#define DEBUG_SERIAL Serial // Установка константы, отвечающей за последовательный порт, подключаемый к компьютеру
#define DXL_SERIAL Serial3 // OpenCM9.04 EXP Board's DXL port Serial. (To use the DXL port on the OpenCM 9.04 board, you must use Serial1 for Serial. And because of the OpenCM 9.04 driver code, you must call Serial1.setDxlMode(true); before dxl.begin();.)

#define DXL_DIR_PIN 22 // Инициализация переменной, отвечащей за номер пина, подключенного к информационному пину приводов манипулятора
#define DXL_PROTOCOL_VERSION 1.0 // Инициализация переменной, отвечащей за протокол передачи данных от OpenCM9.04 к приводам
#define JOINT_N 6 // Количество приводов
#define DYNAMIXEL_GOAL_POS_ERROR 1 // Погрешность позиции для динимикселей

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
  DEBUG_SERIAL.println("Wait press btn1...");
  //while(!DEBUG_SERIAL); // Ждём, пока монитор порта не откроется
  while(true) {
    if (digitalRead(EXP_BOARD_BUTTON1_PIN) == 0) { // Автоматический режим демонтрации
      workMode = 1;
      break; // Кнопка 1 на плате расширения
    }
    if (digitalRead(EXP_BOARD_BUTTON2_PIN) == 0) { // Режим управления
      workMode = 2;
      break; // Кнопка 2 на плате расширения
    }
  }
  DEBUG_SERIAL.println("Setup...");
  dxl.begin(1000000); // Установка скорости обмена данными по последовательному порту манипулятора
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION); // Выбор протокола обмена данными
  for (int i = 1; i <= JOINT_N; i++) { // Цикл для перебора всех приводов
    while (true) {
      if(dxl.ping(i) == true) { // Проверка отвечает ли мотор
        DEBUG_SERIAL.print("Dynamixel width ID "); DEBUG_SERIAL.print(i); DEBUG_SERIAL.print(" found, model "); DEBUG_SERIAL.print(dxl.getModelNumber(i)); DEBUG_SERIAL.println(".");
        break;
      } else {
        DEBUG_SERIAL.print("Dynamixel width ID "); DEBUG_SERIAL.print(i); DEBUG_SERIAL.print(" not found!"); DEBUG_SERIAL.println(" Wait...");
        delay(500);
      }
    }
    dxl.torqueOff(i); // Отключение блокировки привода, чтобы установить режим работы!
    bool setDinamixelOperationMode = dxl.setOperatingMode(i, OP_POSITION); // Установка режима работы привода в качестве шарнира
    if (!setDinamixelOperationMode) {
      DEBUG_SERIAL.print("Dynamixel width ID "); DEBUG_SERIAL.print(i); DEBUG_SERIAL.println("mode not set!");
    }
    delay(10);
  }
  DEBUG_SERIAL.println("Start...");
  DEBUG_SERIAL.println();
  // Занять среднюю позицию всем сервоприводам
  float servosDegPos[] = {512, 512, 512, 512, 512, 512};
  MoveServosToPos(servosDegPos, 3000);
}

void loop() {
  if (workMode == 1) {
    while (true) {
      SetAllServosSpeed(40);
    }
  } else if (workMode == 2) {
    while (true) {
      float x, y, z;
      if (Serial.available() > 2) {
        // Встроенная функция readStringUntil будет читать все данные, пришедшие в UART до специального символа — '\n' (перенос строки).
        // Он появляется в паре с '\r' (возврат каретки) при передаче данных функцией Serial.println().
        // Эти символы удобно передавать для разделения команд, но не очень удобно обрабатывать. Удаляем их функцией trim().
        String command = Serial.readStringUntil('\n');
        command.trim();
        command.replace(" ", ""); // Убрать возможные пробелы между символами
        byte strIndex = command.length(); // Переменая для хронения индекса вхождения цифры в входной строке
        // Поиск первого вхождения цифры от 0 по 9 в подстроку
        for (byte i = 0; i < 10; i++) {
          byte index = command.indexOf(String(i));
          if (index < strIndex && index != 255) strIndex = index;
        }
        String incoming = command.substring(0, strIndex);
        String valueStr = command.substring(strIndex, command.length());
        float value = valueStr.toFloat();
        if (incoming == "x") {
          x = value;
        } else if (incoming == "y") {
          y = value;
        } else if (incoming == "z") {
          z = value;
        } else if (incoming == "s") {
          SetAllServosSpeed(value);
        } else if (incoming == "g") { // Захват
          //if (value == 1) PneumaticSuctionCupState(true, 0);
          //else if (value == 0) PneumaticSuctionCupState(false, 0);
        }
        //if (incoming != "g") DeltaMoveToPos(x, y, z, true); // Занять позицию, если это была не команда управления присоской
      }
    }
  }
}

int ConvertDegreesToGoalPos(float degPos) {
  // 30, 270 - мертвые зоны диномикселя
  degPos = constrain(degPos, 30, 300); // Ограничиваем входное значение, где 30° - это начальный градус слева и 300°
  float goalPos = map(degPos, 330, 30, 1023, 0);
  return goalPos;
}

// Ждать пока сервомоторы не займут позиции
void WaitMotorsTakeGoalPos(float *waitServosPos) { 
  int* servosPos = new int[JOINT_N];
  while (true) {
    servosPos = GetServosPos();
    DEBUG_SERIAL.print("Current servos position: ");
    for (byte i = 0; i < JOINT_N; i++) {
      DEBUG_SERIAL.print(servosPos[i + 1]);
      if (i < JOINT_N) DEBUG_SERIAL.print(", ");
      else DEBUG_SERIAL.println();
    }
    bool servosIsPerformed[JOINT_N];
    for (byte i = 0; i < JOINT_N; i++) { // Проверяем условие и записываем в массив для каждого отдельного серво
      servosIsPerformed[i + 1] = waitServosPos[i] - DYNAMIXEL_GOAL_POS_ERROR <= servosPos[i] && servosPos[i] <= waitServosPos[i] + DYNAMIXEL_GOAL_POS_ERROR;
    }
    if (servosIsPerformed[0] && servosIsPerformed[1] && servosIsPerformed[2]) break; // Если все условия выполнились, то выйти из цикла
    delay(10);
  }
  DEBUG_SERIAL.print("Motors performed position: "); DEBUG_SERIAL.print(dxl.getPresentPosition(1)); DEBUG_SERIAL.print(", "); DEBUG_SERIAL.print(dxl.getPresentPosition(2)); DEBUG_SERIAL.print(", "); DEBUG_SERIAL.println(dxl.getPresentPosition(3));
  for (byte i = 0; i < JOINT_N; i++) {
    DEBUG_SERIAL.print(servosPos[i + 1]);
    if (i < JOINT_N) DEBUG_SERIAL.print(", ");
    else DEBUG_SERIAL.println();
  }
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
void MoveServoToPos(byte servoId, float pos) {
  dxl.setGoalPosition(servoId, pos); // Задание целевого положения
}

// Сервоприводам занять позиции
void MoveServosToPos(float *servosPos, bool waitPerformedPos) {
  for (byte i = 0; i < JOINT_N; i++) {
    dxl.setGoalPosition(i + 1, servosPos[i]); // Задание целевого положения
  }
  if (waitPerformedPos) WaitMotorsTakeGoalPos(servosPos);
}

// Получить от серво его угол
int GetServoPos(byte servoId) {
  return dxl.getPresentPosition(servoId);
}

// Получить значения углов с сервоприводов
int* GetServosPos() {
  int *pos = new int[JOINT_N];
  for (int i = 0; i < JOINT_N; i++) {
    pos[i] = GetServoPos(i + 1);
  }
  return pos;
}

// Функция обратной кинематики
float* Manipulator_IK(float x, float y, float z) {
  float a1 = atan(y / x);
  float a5 = a1;
  float k = sqrt (x * x) + (y * y);
  float z_solve = z + LINK4 - LINK1;
  float d = sqrt (k * k) + (z_solve * z_solve);
  float a2 = (3.14 / 2) - (atan(z_solve / k) + acos((d * d) + (LINK2 * LINK2) - (LINK3 * LINK3)) / (2 *  d * LINK2));
  float a3 = 3.14 - acos(((-d * -d) + (LINK2 * LINK2) - (LINK3 * LINK3)) / (2 * LINK2 * LINK3));
  float a4 = 3.14 - (a2 - a3);
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
