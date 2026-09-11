#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled!
#endif

BluetoothSerial SerialBT;
Adafruit_ICM20948 icm;

const unsigned long INTERVALO_MS = 10; 
unsigned long tiempoAnterior = 0;

// Variables para almacenar el error de reposo del giroscopio
float offset_gx = 0.0;
float offset_gy = 0.0;
float offset_gz = 0.0;

void setup(void) {
  Serial.begin(115200);
  SerialBT.begin("ESP32-Seguidor-1"); // Etiqueta Bluetooth

  if (!icm.begin_I2C()) {
    while (1) { delay(10); } 
  }

  icm.setAccelRange(ICM20948_ACCEL_RANGE_8_G);
  icm.setGyroRange(ICM20948_GYRO_RANGE_1000_DPS);

  // --- RUTINA DE CALIBRACIÓN DEL GIROSCOPIO ---
  delay(2000); // 2 segundos para soltar el sensor y evitar vibraciones de la mano
  
  const int muestras = 500;
  for (int i = 0; i < muestras; i++) {
    sensors_event_t accel, gyro, temp, mag;
    icm.getEvent(&accel, &gyro, &temp, &mag);
    
    offset_gx += gyro.gyro.x;
    offset_gy += gyro.gyro.y;
    offset_gz += gyro.gyro.z;
    
    delay(5); // Pequeña pausa para no saturar el bus I2C durante la lectura rápida
  }
  
  // Promediar el error acumulado
  offset_gx /= muestras;
  offset_gy /= muestras;
  offset_gz /= muestras;
  // ---------------------------------------------

  tiempoAnterior = millis();
}

void loop() {
  unsigned long tiempoActual = millis();

  if (tiempoActual - tiempoAnterior >= INTERVALO_MS) {
    tiempoAnterior = tiempoActual;

    sensors_event_t accel, gyro, temp, mag;
    if (!icm.getEvent(&accel, &gyro, &temp, &mag)) return;

    // Restar el offset calculado para obtener ceros perfectos en reposo
    float gx_corregido = gyro.gyro.x - offset_gx;
    float gy_corregido = gyro.gyro.y - offset_gy;
    float gz_corregido = gyro.gyro.z - offset_gz;

    // Formato crudo de 10 valores
    String paquete = String(tiempoActual) + "," + 
                     String(accel.acceleration.x, 3) + "," + 
                     String(accel.acceleration.y, 3) + "," + 
                     String(accel.acceleration.z, 3) + "," + 
                     String(gx_corregido * 57.2958, 2) + "," + // Uso de los valores corregidos
                     String(gy_corregido * 57.2958, 2) + "," + 
                     String(gz_corregido * 57.2958, 2) + "," + 
                     String(mag.magnetic.x, 2) + "," + 
                     String(mag.magnetic.y, 2) + "," + 
                     String(mag.magnetic.z, 2);

    SerialBT.println(paquete);
  }
}