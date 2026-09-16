import serial
import numpy as np
from ahrs.filters import Madgwick
from scipy.spatial.transform import Rotation as R
import opensim as osim 
import threading
import time
import json
import os
import sys

PUERTO_MUSLO = 'COM4'
PUERTO_PANTORRILLA = 'COM9'
BAUDRATE = 115200
FRECUENCIA = 100.0
MUESTRAS_CALIBRACION = 12000
ARCHIVO_DATOS = "datos_sujeto.json"

def gestionar_datos_sujeto():
    datos = {}
    if os.path.exists(ARCHIVO_DATOS):
        try:
            with open(ARCHIVO_DATOS, 'r') as f:
                datos = json.load(f)
            sexo_str = "Hombre" if datos.get('sexo') == 'h' else "Mujer" if datos.get('sexo') == 'm' else "N/A"
            print("\n" + "="*40)
            print(" PERFIL DE SUJETO ENCONTRADO")
            print("="*40)
            print(f" Sexo:   {sexo_str}")
            print(f" Peso:   {datos.get('peso', 'N/A')} kg")
            print(f" Altura: {datos.get('altura', 'N/A')} m")
            
            if 'muslo' in datos:
                print(" [✓] Parámetros inerciales 3D precalculados en caché.")
            print("="*40)
            
            respuesta = input("\n¿Deseas utilizar estos datos para el análisis dinámico? (s/n): ").strip().lower()
            if respuesta == 's':
                return datos
        except json.JSONDecodeError:
            pass

    print("\n" + "="*40)
    print(" NUEVO PERFIL DE SUJETO")
    print("="*40)
    while True:
        try:
            peso = float(input(" Ingresa el peso del sujeto (en kg): "))
            altura = float(input(" Ingresa la altura del sujeto (en metros): "))
            while True:
                sexo = input(" Ingresa el sexo del sujeto (h para hombre / m para mujer): ").strip().lower()
                if sexo in ['h', 'm']:
                    break
                print(" [Error] Por favor, ingresa 'h' o 'm'.")
            break
        except ValueError:
            print(" [Error] Por favor, ingresa valores numéricos válidos.")
    
    L_muslo = altura * 0.245
    L_pant = altura * 0.246
    
    if sexo == 'h':
        m_muslo, m_pant = peso * 0.1416, peso * 0.0433
        k_muslo_x, k_muslo_y, k_muslo_z = 0.329, 0.329, 0.149
        k_pant_x, k_pant_y, k_pant_z = 0.251, 0.246, 0.102
    else: 
        m_muslo, m_pant = peso * 0.1478, peso * 0.0481
        k_muslo_x, k_muslo_y, k_muslo_z = 0.369, 0.364, 0.162
        k_pant_x, k_pant_y, k_pant_z = 0.267, 0.263, 0.092
    
    I_muslo = {
        "Ixx": m_muslo * (k_muslo_x * L_muslo)**2,
        "Iyy": m_muslo * (k_muslo_y * L_muslo)**2,
        "Izz": m_muslo * (k_muslo_z * L_muslo)**2
    }
    
    I_pant = {
        "Ixx": m_pant * (k_pant_x * L_pant)**2,
        "Iyy": m_pant * (k_pant_y * L_pant)**2,
        "Izz": m_pant * (k_pant_z * L_pant)**2
    }
    
    datos = {
        'peso': peso, 'altura': altura, 'sexo': sexo,
        'muslo': {'masa': m_muslo, 'longitud': L_muslo, 'inercia': I_muslo},
        'pantorrilla': {'masa': m_pant, 'longitud': L_pant, 'inercia': I_pant}
    }
    
    with open(ARCHIVO_DATOS, 'w') as f:
        json.dump(datos, f, indent=4)
        
    print("\n[Sistema] Datos inerciales calculados y guardados exitosamente.")
    return datos

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
            
            print(f"[{nombre}] Estabilizando filtro IMU...")
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
                        a_atr, g_atr, m_atr = parse_line(ser.readline())
                        if a_atr is not None:
                            q_imu = madgwick.updateMARG(q_imu, gyr=g_atr, acc=a_atr, mag=m_atr)
                    
                    q_scipy = np.array([q_imu[1], q_imu[2], q_imu[3], q_imu[0]])
                    buffer_calibracion.append(q_scipy)
                    print(f"[{nombre}] Calibrando: {len(buffer_calibracion)}/{MUESTRAS_CALIBRACION}    ", end='\r')
            
            ser.close()
            q_init_avg = np.mean(buffer_calibracion, axis=0)
            q_init_avg /= np.linalg.norm(q_init_avg)
            print(f"\n[{nombre}] ¡Calibración completada con éxito!")
            return R.from_quat(q_init_avg).inv(), q_imu 
            
        except Exception as e:
            time.sleep(5)

class LectorSensor(threading.Thread):
    def __init__(self, puerto, nombre, r_calibracion, q_imu_estabilizado, altura, sexo, es_muslo=True):
        super().__init__()
        self.puerto = puerto
        self.nombre = nombre
        self.r_calibracion = r_calibracion
        self.madgwick = Madgwick(sampleperiod=1.0/FRECUENCIA)
        self.q_imu = np.copy(q_imu_estabilizado) 
        self.rotacion_relativa = None
        
        self.gyro_actual = np.zeros(3)
        self.gyro_anterior = np.zeros(3)
        self.aceleracion_angular = np.zeros(3)
        self.aceleracion_scom = np.zeros(3)
        
        if es_muslo:
            l_seg = 0.245 * altura
            scom_offset = 0.4095 if sexo == 'h' else 0.3612
            y_offset = (0.50 - scom_offset) * l_seg
            x_offset = -0.06 
        else:
            l_seg = 0.246 * altura
            scom_offset = 0.4395 if sexo == 'h' else 0.4352
            y_offset = (0.50 - scom_offset) * l_seg
            x_offset = -0.04 
            
        self.r_vector = np.array([x_offset, y_offset, 0.0])
        self.ejecutando = True
        self.daemon = True

    def run(self):
        while self.ejecutando:
            try:
                ser = serial.Serial(self.puerto, BAUDRATE, timeout=1)
                while self.ejecutando:
                    line = ser.readline()
                    if not line: continue
                    accel, gyro, mag = parse_line(line)
                    
                    if accel is not None:
                        self.gyro_actual = np.copy(gyro)
                        self.aceleracion_angular = (gyro - self.gyro_anterior) * FRECUENCIA
                        self.gyro_anterior = np.copy(gyro)
                        
                        term_tangencial = np.cross(self.aceleracion_angular, self.r_vector)
                        term_centripeto = np.cross(gyro, np.cross(gyro, self.r_vector))
                        self.aceleracion_scom = accel + term_tangencial + term_centripeto
                        
                        self.q_imu = self.madgwick.updateMARG(self.q_imu, gyr=gyro, acc=accel, mag=mag)
                        while ser.in_waiting > 50: 
                            a_atr, g_atr, m_atr = parse_line(ser.readline())
                            if a_atr is not None:
                                self.q_imu = self.madgwick.updateMARG(self.q_imu, gyr=g_atr, acc=a_atr, mag=m_atr)
                        
                        q_scipy = np.array([self.q_imu[1], self.q_imu[2], self.q_imu[3], self.q_imu[0]])
                        self.rotacion_relativa = self.r_calibracion * R.from_quat(q_scipy)

                ser.close()
            except Exception as e:
                if self.ejecutando: time.sleep(3)

    def get_datos_cinematicos(self):
        return self.rotacion_relativa, self.aceleracion_angular, self.aceleracion_scom, self.gyro_actual
        
    def detener(self):
        self.ejecutando = False

def main():
    datos = gestionar_datos_sujeto()
    
    I_m = np.diag([datos['muslo']['inercia']['Ixx'], datos['muslo']['inercia']['Iyy'], datos['muslo']['inercia']['Izz']])
    I_p = np.diag([datos['pantorrilla']['inercia']['Ixx'], datos['pantorrilla']['inercia']['Iyy'], datos['pantorrilla']['inercia']['Izz']])
    m_muslo, m_pant = datos['muslo']['masa'], datos['pantorrilla']['masa']
    L_muslo, L_pant = datos['muslo']['longitud'], datos['pantorrilla']['longitud']
    
    scom_pct_m = 0.4095 if datos['sexo'] == 'h' else 0.3612
    scom_pct_p = 0.4395 if datos['sexo'] == 'h' else 0.4352
    
    print("\nMantenga los sensores estáticos para la calibración inicial...")
    r_cal_pantorrilla, q_init_pantorrilla = calibrar_sensor(PUERTO_PANTORRILLA, "PANTORRILLA")
    r_cal_muslo, q_init_muslo = calibrar_sensor(PUERTO_MUSLO, "MUSLO")
    
    osim.ModelVisualizer.addDirToGeometrySearchPaths("../opensim_tools/geometry")
    modelo = osim.Model("../opensim_tools/GaitModel.osim") 
    modelo.setUseVisualizer(True)
    estado = modelo.initSystem()
    coord_set = modelo.getCoordinateSet()
    
    c_flex_m = coord_set.get("hip_flexion_r") if coord_set.contains("hip_flexion_r") else None
    c_add_m = coord_set.get("hip_adduction_r") if coord_set.contains("hip_adduction_r") else None
    c_rot_m = coord_set.get("hip_rotation_r") if coord_set.contains("hip_rotation_r") else None
    c_rod = coord_set.get("knee_angle_r") if coord_set.contains("knee_angle_r") else None

    hilo_muslo = LectorSensor(PUERTO_MUSLO, "MUSLO", r_cal_muslo, q_init_muslo, datos['altura'], datos['sexo'], es_muslo=True)
    hilo_pantorrilla = LectorSensor(PUERTO_PANTORRILLA, "PANTORRILLA", r_cal_pantorrilla, q_init_pantorrilla, datos['altura'], datos['sexo'], es_muslo=False)
    
    hilo_muslo.start()
    hilo_pantorrilla.start()
    
    os.system('cls' if os.name == 'nt' else 'clear')
    
    try:
        while True:
            # Reasignación de nombres exactos para mantener la impresión original
            r_muslo, alpha_muslo, a_scom_M, gyro_M = hilo_muslo.get_datos_cinematicos()
            r_pantorrilla, alpha_pantorrilla, a_scom_P, gyro_P = hilo_pantorrilla.get_datos_cinematicos()

            if r_muslo is not None and r_pantorrilla is not None:
                ang_muslo = r_muslo.as_euler('xyz', degrees=False)
                ang_pant = r_pantorrilla.as_euler('xyz', degrees=False)
                
                flexion_hip = -ang_muslo[1]   
                adduccion_hip = ang_muslo[2]  
                rotacion_hip = -ang_muslo[0]  
                flexion_knee = min(0.0, -ang_pant[1] - flexion_hip)

                # --- PANTORRILLA (Bottom) ---
                tau_euler_P = np.dot(I_p, alpha_pantorrilla) + np.cross(gyro_P, np.dot(I_p, gyro_P))
                F_rodilla = m_pant * a_scom_P 
                r_CoM_a_Rodilla_P = np.array([0, scom_pct_p * L_pant, 0])
                tau_rodilla = tau_euler_P + np.cross(r_CoM_a_Rodilla_P, F_rodilla)
                
                # --- TRANSFERENCIA ---
                Rot_Relativa_P_a_M = r_muslo.inv() * r_pantorrilla
                F_rodilla_en_Muslo = Rot_Relativa_P_a_M.apply(F_rodilla)
                tau_rodilla_en_Muslo = Rot_Relativa_P_a_M.apply(tau_rodilla)
                
                # --- MUSLO (Up) ---
                tau_euler_M = np.dot(I_m, alpha_muslo) + np.cross(gyro_M, np.dot(I_m, gyro_M))
                F_cadera = (m_muslo * a_scom_M) + F_rodilla_en_Muslo
                r_CoM_a_Cadera_M = np.array([0, scom_pct_m * L_muslo, 0])
                r_CoM_a_Rodilla_M = np.array([0, -(1 - scom_pct_m) * L_muslo, 0]) 
                
                # Este es el Torque Total (Cadera absorbe el esfuerzo local propio + el de la rodilla)
                tau_cadera = tau_euler_M + np.cross(r_CoM_a_Cadera_M, F_cadera) - np.cross(r_CoM_a_Rodilla_M, F_rodilla_en_Muslo) + tau_rodilla_en_Muslo
                
                if c_flex_m: c_flex_m.setValue(estado, flexion_hip)
                if c_add_m: c_add_m.setValue(estado, adduccion_hip)
                if c_rot_m: c_rot_m.setValue(estado, rotacion_hip)
                if c_rod: c_rod.setValue(estado, flexion_knee)
                modelo.realizePosition(estado)
                modelo.getVisualizer().show(estado)
                
                # --- HUD EN TERMINAL ---
                str_hip = f"Hip [Flex: {np.degrees(flexion_hip):5.1f}°, Add: {np.degrees(adduccion_hip):5.1f}°, Rot: {np.degrees(rotacion_hip):5.1f}°] | Knee Flex: {np.degrees(flexion_knee):5.1f}°"
                str_am = f"[{alpha_muslo[0]:5.1f}, {alpha_muslo[1]:5.1f}, {alpha_muslo[2]:5.1f} ]"
                str_ap = f"[{alpha_pantorrilla[0]:5.1f}, {alpha_pantorrilla[1]:5.1f}, {alpha_pantorrilla[2]:5.1f} ]"
                str_alpha = f"α Muslo: {str_am} | α Pant: {str_ap} rad/s²"
                str_tau = f"Torque Flex/Ext (Nm) -> TOTAL (Cadera): {tau_cadera[0]:6.2f} | SUB-TOTAL (Rodilla): {tau_rodilla[0]:6.2f}"
                

                sys.stdout.write(f"{str_hip}\n{str_alpha}\n{str_tau}\033[F\033[F")
                sys.stdout.flush()
            
            time.sleep(0.015)

    except KeyboardInterrupt:
        print("\n\n\n\nCaptura detenida por el usuario. Cerrando conexiones...")
    finally:
        hilo_muslo.detener()
        hilo_pantorrilla.detener()
        hilo_muslo.join(timeout=1.0)
        hilo_pantorrilla.join(timeout=1.0)

if __name__ == "__main__":
    main()