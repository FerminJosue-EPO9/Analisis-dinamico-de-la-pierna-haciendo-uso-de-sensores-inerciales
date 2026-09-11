import serial
import numpy as np
from ahrs.filters import Madgwick
from scipy.spatial.transform import Rotation as R
import opensim as osim 
import threading
import time

PUERTO_MUSLO = 'COM4'
PUERTO_PANTORRILLA = 'COM9'
BAUDRATE = 115200
FRECUENCIA = 100.0
MUESTRAS_CALIBRACION = 12000

def parse_line(line):
    parts = line.decode('utf-8', errors='ignore').strip().split(',')
    if len(parts) == 10:
        try:
            data = np.array([float(x) for x in parts[1:]])
            accel = data[0:3]
            gyro = data[3:6] * (np.pi / 180.0) 
            mag = data[6:9]
            return accel, gyro, mag
        except ValueError:
            pass
    return None, None, None

def calibrar_sensor(puerto, nombre):
    madgwick = Madgwick(sampleperiod=1.0/FRECUENCIA)
    q_imu = np.array([1.0, 0.0, 0.0, 0.0])
    buffer_calibracion = []
    
    while True:
        try:
            print(f"\nIniciando conexión con {nombre} ({puerto})...")
            ser = serial.Serial(puerto, BAUDRATE, timeout=1) 
            ser.reset_input_buffer() 
            
            print(f"[{nombre}] Estabilizando filtro IMU (Warm-up)...")
            for _ in range(500):
                line = ser.readline()
                if not line: continue
                accel, gyro, mag = parse_line(line)
                if accel is not None:
                    q_imu = madgwick.updateMARG(q_imu, gyr=gyro, acc=accel, mag=mag)
            
            print(f"[{nombre}] Iniciando recolección de calibración...")
            buffer_calibracion.clear()
            
            while len(buffer_calibracion) < MUESTRAS_CALIBRACION:
                line = ser.readline()
                if not line: continue
                
                accel, gyro, mag = parse_line(line)
                
                if accel is not None:
                    q_imu = madgwick.updateMARG(q_imu, gyr=gyro, acc=accel, mag=mag)
                    
                    while ser.in_waiting > 50: 
                        line_atrasada = ser.readline()
                        a_atr, g_atr, m_atr = parse_line(line_atrasada)
                        if a_atr is not None:
                            q_imu = madgwick.updateMARG(q_imu, gyr=g_atr, acc=a_atr, mag=m_atr)
                    
                    q_scipy = np.array([q_imu[1], q_imu[2], q_imu[3], q_imu[0]])
                    buffer_calibracion.append(q_scipy)
                    
                    print(f"[{nombre}] Calibrando: {len(buffer_calibracion)}/{MUESTRAS_CALIBRACION}    ", end='\r')
            
            ser.close()
            q_init_avg = np.mean(buffer_calibracion, axis=0)
            q_init_avg /= np.linalg.norm(q_init_avg)
            r_calibracion = R.from_quat(q_init_avg).inv()
            
            print(f"\n[{nombre}] ¡Calibración completada con éxito!")
            return r_calibracion, q_imu 
            
        except Exception as e:
            print("\nHa ocurrido un problema durante la recolección de lecturas, intentandolo de nuevo...")
            time.sleep(5)

class LectorSensor(threading.Thread):
    def __init__(self, puerto, nombre, r_calibracion, q_imu_estabilizado):
        super().__init__()
        self.puerto = puerto
        self.nombre = nombre
        self.r_calibracion = r_calibracion
        self.madgwick = Madgwick(sampleperiod=1.0/FRECUENCIA)
        self.q_imu = np.copy(q_imu_estabilizado) 
        self.rotacion_relativa = None
        self.ejecutando = True
        self.daemon = True

    def run(self):
        while self.ejecutando:
            try:
                ser = serial.Serial(self.puerto, BAUDRATE, timeout=1)
                print(f"\n[{self.nombre}] Puerto {self.puerto} conectado correctamente.")
                
                while self.ejecutando:
                    line = ser.readline()
                    if not line: 
                        continue
                    
                    accel, gyro, mag = parse_line(line)
                    
                    if accel is not None:
                        self.q_imu = self.madgwick.updateMARG(self.q_imu, gyr=gyro, acc=accel, mag=mag)
                        
                        while ser.in_waiting > 50: 
                            line_atrasada = ser.readline()
                            a_atr, g_atr, m_atr = parse_line(line_atrasada)
                            if a_atr is not None:
                                self.q_imu = self.madgwick.updateMARG(self.q_imu, gyr=g_atr, acc=a_atr, mag=m_atr)
                        
                        q_scipy = np.array([self.q_imu[1], self.q_imu[2], self.q_imu[3], self.q_imu[0]])
                        self.rotacion_relativa = self.r_calibracion * R.from_quat(q_scipy)

                ser.close()

            except Exception as e:
                if self.ejecutando:
                    print(f"\n[{self.nombre}] Fallo de comunicación en {self.puerto}. Reintentando en 3s... ({e})")
                    time.sleep(3)

    def get_rotacion(self):
        return self.rotacion_relativa
        
    def detener(self):
        self.ejecutando = False

def main():
    print("Mantenga los sensores estáticos para la calibración inicial...")
    
    # 1. Calibración Secuencial
    r_cal_pantorrilla, q_init_pantorrilla = calibrar_sensor(PUERTO_PANTORRILLA, "PANTORRILLA")
    r_cal_muslo, q_init_muslo = calibrar_sensor(PUERTO_MUSLO, "MUSLO")
    
    print("\nCargando modelo biomecánico en OpenSim...")
    osim.ModelVisualizer.addDirToGeometrySearchPaths("../opensim_tools/geometry")
    modelo = osim.Model("../opensim_tools/GaitModel.osim") 
    modelo.setUseVisualizer(True)
    estado = modelo.initSystem()

    coord_set = modelo.getCoordinateSet()
    coord_flex_muslo = coord_set.get("hip_flexion_r") if coord_set.contains("hip_flexion_r") else None
    coord_add_muslo = coord_set.get("hip_adduction_r") if coord_set.contains("hip_adduction_r") else None
    coord_rot_muslo = coord_set.get("hip_rotation_r") if coord_set.contains("hip_rotation_r") else None
    coord_rodilla = coord_set.get("knee_angle_r") if coord_set.contains("knee_angle_r") else None

    # 2. Inicio del procesamiento multihilo post-calibración
    print("\nIniciando hilos de sensores para renderizado en tiempo real...")
    hilo_muslo = LectorSensor(PUERTO_MUSLO, "MUSLO", r_cal_muslo, q_init_muslo)
    hilo_pantorrilla = LectorSensor(PUERTO_PANTORRILLA, "PANTORRILLA", r_cal_pantorrilla, q_init_pantorrilla)
    
    hilo_muslo.start()
    hilo_pantorrilla.start()

    try:
        while True:
            r_muslo = hilo_muslo.get_rotacion()
            r_pantorrilla = hilo_pantorrilla.get_rotacion()

            if r_muslo is not None and r_pantorrilla is not None:
                ang_muslo = r_muslo.as_euler('xyz', degrees=False)
                ang_pantorrilla = r_pantorrilla.as_euler('xyz', degrees=False)
                
                flexion_hip = -ang_muslo[1]   
                adduccion_hip = ang_muslo[2]  
                rotacion_hip = -ang_muslo[0]  

                flexion_pantorrilla = -ang_pantorrilla[1]
                flexion_knee = flexion_pantorrilla - flexion_hip
                
                if flexion_knee > 0:
                    flexion_knee = 0.0

                if coord_flex_muslo: coord_flex_muslo.setValue(estado, flexion_hip)
                if coord_add_muslo: coord_add_muslo.setValue(estado, adduccion_hip)
                if coord_rot_muslo: coord_rot_muslo.setValue(estado, rotacion_hip)
                if coord_rodilla: coord_rodilla.setValue(estado, flexion_knee)
                
                modelo.realizePosition(estado)
                modelo.getVisualizer().show(estado)
                
                # --- LÍNEA DE IMPRESIÓN ACTUALIZADA ---
                print(f"Hip [Flex: {np.degrees(flexion_hip):.1f}°, Add: {np.degrees(adduccion_hip):.1f}°, Rot: {np.degrees(rotacion_hip):.1f}°] | Knee Flex: {np.degrees(flexion_knee):.1f}°    ", end='\r')
            
            time.sleep(0.015)

    except KeyboardInterrupt:
        print("\n\nCaptura detenida por el usuario. Cerrando conexiones...")
    finally:
        hilo_muslo.detener()
        hilo_pantorrilla.detener()
        hilo_muslo.join(timeout=1.0)
        hilo_pantorrilla.join(timeout=1.0)

if __name__ == "__main__":
    main()