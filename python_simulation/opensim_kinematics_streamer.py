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
                print(" [✓] Parámetros inerciales 3D y de escalado precalculados en caché.")
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
        "Iyy": m_muslo * (k_muslo_z * L_muslo)**2,
        "Izz": m_muslo * (k_muslo_y * L_muslo)**2
    }
    
    I_pant = {
        "Ixx": m_pant * (k_pant_x * L_pant)**2,
        "Iyy": m_pant * (k_pant_z * L_pant)**2,
        "Izz": m_pant * (k_pant_y * L_pant)**2
    }

    L_MODELO_FEMUR = 0.3958
    L_MODELO_TIBIA = 0.4300

    L_SUJETO_FEMUR = altura * 0.245
    L_SUJETO_TIBIA = altura * 0.246

    factor_femur = L_SUJETO_FEMUR / L_MODELO_FEMUR
    factor_tibia = L_SUJETO_TIBIA / L_MODELO_TIBIA

    factores_escala = {
        "femur": factor_femur,
        "tibia": factor_tibia,
        "pie": factor_tibia,
        "pelvis": factor_femur,
        "torso": (factor_femur + factor_tibia) / 2.0,
    }

    com_muslo = (0.4095 if sexo == 'h' else 0.3612) * L_SUJETO_FEMUR
    com_pant  = (0.4395 if sexo == 'h' else 0.4352) * L_SUJETO_TIBIA

    datos = {
        'peso': peso, 'altura': altura, 'sexo': sexo,
        'muslo': {
            'masa': m_muslo,
            'longitud': L_SUJETO_FEMUR,
            'longitud_modelo': L_MODELO_FEMUR,
            'inercia': I_muslo,
            'com_y': -com_muslo,           
            'factor_escala': factores_escala['femur']
        },
        'pantorrilla': {
            'masa': m_pant,
            'longitud': L_SUJETO_TIBIA,
            'longitud_modelo': L_MODELO_TIBIA,
            'inercia': I_pant,
            'com_y': -com_pant,
            'factor_escala': factores_escala['tibia']
        },
        'factores_escala_globales': factores_escala,
        'modelo_escalado': 'modelo_sujeto_escalado.osim'   
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
    def __init__(self, puerto, nombre, r_calibracion, q_imu_estabilizado, datos, es_muslo=True):
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
            l_seg = datos['muslo']['longitud']
            com_y = datos['muslo']['com_y']
            y_offset = (0.5 * l_seg) + com_y
            x_offset = -0.06
        else:
            l_seg = datos['pantorrilla']['longitud']
            com_y = datos['pantorrilla']['com_y']
            y_offset = (0.5 * l_seg) + com_y
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

                        self.q_imu = self.madgwick.updateMARG(self.q_imu, gyr=gyro, acc=accel, mag=mag)
                        q_scipy_actual = np.array([self.q_imu[1], self.q_imu[2], self.q_imu[3], self.q_imu[0]])
                        rotacion_actual = R.from_quat(q_scipy_actual)

                        gravedad_global = np.array([0.0, 0.0, 9.81])

                        gravedad_local = rotacion_actual.inv().apply(gravedad_global)

                        accel_inercial = accel - gravedad_local

                        term_tangencial = np.cross(self.aceleracion_angular, self.r_vector)
                        term_centripeto = np.cross(gyro, np.cross(gyro, self.r_vector))
                        self.aceleracion_scom = accel_inercial + term_tangencial + term_centripeto
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

def exportar_a_mot(archivo_salida, buffer_datos):
    num_rows = len(buffer_datos['time'])
    num_cols = 10  
    
    with open(archivo_salida, 'w') as f:
        f.write("Coordinates\n")
        f.write("version=1\n")
        f.write(f"nRows={num_rows}\n")
        f.write(f"nColumns={num_cols}\n")
        f.write("inDegrees=yes\n")
        f.write("endheader\n")
        
        f.write("time\tpelvis_ty\t"
                "hip_flexion_r\thip_adduction_r\thip_rotation_r\tknee_angle_r\t"
                "hip_flexion_l\thip_adduction_l\thip_rotation_l\tknee_angle_l\n")
        
        for i in range(num_rows):
            t = buffer_datos['time'][i]
            hf = np.degrees(buffer_datos['hip_flex'][i])
            ha = np.degrees(buffer_datos['hip_add'][i])
            hr = np.degrees(buffer_datos['hip_rot'][i])
            kf = np.degrees(buffer_datos['knee_flex'][i])
            
            f.write(f"{t:.4f}\t0.9000\t"
                    f"{hf:.4f}\t{ha:.4f}\t{hr:.4f}\t{kf:.4f}\t"
                    f"0.0000\t0.0000\t0.0000\t0.0000\n")
            
    print(f"\n[Exito] {num_rows} frames exportados exitosamente a {archivo_salida}")

def generar_modelo_escalado(datos):

    carpeta_modelos = os.path.abspath("../opensim_tools")
    ruta_modelo_gen = os.path.join(carpeta_modelos, "GaitModel.osim")
    ruta_modelo_esc = os.path.join(carpeta_modelos, datos['modelo_escalado'])
    ruta_setup_xml = os.path.join(carpeta_modelos, "Scale_Setup.xml")

    if not os.path.exists(ruta_modelo_gen):
        print(f"[ERROR] No se encuentra el modelo genérico en: {ruta_modelo_gen}")
        print("       Verifica la ruta y la existencia del archivo.")
        raise FileNotFoundError(ruta_modelo_gen)

    print(f"[Escalado] Modelo genérico encontrado: {ruta_modelo_gen}")
    print(f"[Escalado] Modelo escalado se guardará en: {ruta_modelo_esc}")

    print("\n" + "="*50)
    print(" GENERANDO MODELO ESCALADO")
    print("="*50)

    if os.path.exists(ruta_modelo_esc):
        print(f"[Escalado] El modelo '{datos['modelo_escalado']}' ya existe.")
        resp = input("¿Deseas regenerarlo? (s/n): ").strip().lower()
        if resp != 's':
            print("[Escalado] Usando modelo existente.")
            return ruta_modelo_esc

    f = datos['factores_escala_globales']
    f_femur  = f['femur']
    f_tibia  = f['tibia']
    f_pie    = f['pie']
    f_pelvis = f['pelvis']
    f_torso  = f['torso']

    xml_content = f"""<?xml version="1.0" encoding="UTF-8" ?>
<OpenSimDocument Version="40600">
    <ScaleTool name="scale_sujeto">
        <mass>{datos['peso']}</mass>
        <height>{datos['altura']}</height>
        <GenericModelMaker>
            <model_file>{ruta_modelo_gen}</model_file>
        </GenericModelMaker>
        <ModelScaler>
            <apply>true</apply>
            <scaling_order> manualScale </scaling_order>
            <ScaleSet name="manual_scales">
                <objects>
                    <Scale name="pelvis">
                        <scales>{f_pelvis} {f_pelvis} {f_pelvis}</scales>
                        <segment>pelvis</segment>
                    </Scale>
                    <Scale name="femur_r">
                        <scales>{f_femur} {f_femur} {f_femur}</scales>
                        <segment>femur_r</segment>
                    </Scale>
                    <Scale name="femur_l">
                        <scales>{f_femur} {f_femur} {f_femur}</scales>
                        <segment>femur_l</segment>
                    </Scale>
                    <Scale name="tibia_r">
                        <scales>{f_tibia} {f_tibia} {f_tibia}</scales>
                        <segment>tibia_r</segment>
                    </Scale>
                    <Scale name="tibia_l">
                        <scales>{f_tibia} {f_tibia} {f_tibia}</scales>
                        <segment>tibia_l</segment>
                    </Scale>
                    <Scale name="talus_r">
                        <scales>{f_pie} {f_pie} {f_pie}</scales>
                        <segment>talus_r</segment>
                    </Scale>
                    <Scale name="talus_l">
                        <scales>{f_pie} {f_pie} {f_pie}</scales>
                        <segment>talus_l</segment>
                    </Scale>
                    <Scale name="calcn_r">
                        <scales>{f_pie} {f_pie} {f_pie}</scales>
                        <segment>calcn_r</segment>
                    </Scale>
                    <Scale name="calcn_l">
                        <scales>{f_pie} {f_pie} {f_pie}</scales>
                        <segment>calcn_l</segment>
                    </Scale>
                    <Scale name="toes_r">
                        <scales>{f_pie} {f_pie} {f_pie}</scales>
                        <segment>toes_r</segment>
                    </Scale>
                    <Scale name="toes_l">
                        <scales>{f_pie} {f_pie} {f_pie}</scales>
                        <segment>toes_l</segment>
                    </Scale>
                    <Scale name="torso">
                        <scales>{f_torso} {f_torso} {f_torso}</scales>
                        <segment>torso</segment>
                    </Scale>
                </objects>
                <groups />
            </ScaleSet>
            <preserve_mass_distribution>true</preserve_mass_distribution>
            <output_model_file>{ruta_modelo_esc}</output_model_file>
        </ModelScaler>
        <MarkerPlacer>
            <apply>false</apply>
        </MarkerPlacer>
    </ScaleTool>
</OpenSimDocument>
"""

    with open(ruta_setup_xml, 'w') as f_xml:
        f_xml.write(xml_content)

    print(f"[Escalado] Configuración escrita en: {ruta_setup_xml}")
    print(f"[Escalado] Ejecutando osim.ScaleTool...")

    try:
        scale_tool = osim.ScaleTool(ruta_setup_xml)
        scale_tool.run()
        print(f"[Escalado] Modelo escalado generado: {ruta_modelo_esc}")
    except Exception as e:
        print(f"[ERROR] Falló ScaleTool: {e}")
        return ruta_modelo_gen

    print("[Escalado] Aplicando masa, CoM e inercia exactos...")
    modelo_esc = osim.Model(ruta_modelo_esc)
    estado_esc = modelo_esc.initSystem()
    cuerpos = modelo_esc.getBodySet()

    m_muslo = datos['muslo']['masa']
    m_pant  = datos['pantorrilla']['masa']
    com_m   = datos['muslo']['com_y']
    com_p   = datos['pantorrilla']['com_y']
    I_m     = datos['muslo']['inercia']
    I_p     = datos['pantorrilla']['inercia']

    for lado in ['r', 'l']:
        femur = cuerpos.get(f"femur_{lado}")
        femur.set_mass(m_muslo)
        femur.set_mass_center(osim.Vec3(0, com_m, 0))
        femur.set_inertia(osim.Vec6(I_m['Ixx'], I_m['Iyy'], I_m['Izz'], 0, 0, 0))
        
        tibia = cuerpos.get(f"tibia_{lado}")
        tibia.set_mass(m_pant)
        tibia.set_mass_center(osim.Vec3(0, com_p, 0))
        tibia.set_inertia(osim.Vec6(I_p['Ixx'], I_p['Iyy'], I_p['Izz'], 0, 0, 0))

    estado_esc = modelo_esc.initSystem()
    modelo_esc.printToXML(ruta_modelo_esc)
    print(f"[Escalado] Modelo final guardado con masa, CoM e inercia sobrescritos.")
    print("="*50 + "\n")

    return ruta_modelo_esc

def main():
    datos = gestionar_datos_sujeto()
    
    I_m = np.diag([datos['muslo']['inercia']['Ixx'], datos['muslo']['inercia']['Iyy'], datos['muslo']['inercia']['Izz']])
    I_p = np.diag([datos['pantorrilla']['inercia']['Ixx'], datos['pantorrilla']['inercia']['Iyy'], datos['pantorrilla']['inercia']['Izz']])
    m_muslo, m_pant = datos['muslo']['masa'], datos['pantorrilla']['masa']
    L_muslo, L_pant = datos['muslo']['longitud'], datos['pantorrilla']['longitud']
    
    scom_pct_m = 0.4095 if datos['sexo'] == 'h' else 0.3612
    scom_pct_p = 0.4395 if datos['sexo'] == 'h' else 0.4352
    
    ruta_modelo_escalado = generar_modelo_escalado(datos)
    
    print("\nMantenga los sensores estáticos para la calibración inicial...")
    r_cal_pantorrilla, q_init_pantorrilla = calibrar_sensor(PUERTO_PANTORRILLA, "PANTORRILLA")
    r_cal_muslo, q_init_muslo = calibrar_sensor(PUERTO_MUSLO, "MUSLO")
    
    osim.ModelVisualizer.addDirToGeometrySearchPaths("../opensim_tools/geometry")
    modelo = osim.Model(ruta_modelo_escalado)
    modelo.setUseVisualizer(True)
    estado = modelo.initSystem()
    coord_set = modelo.getCoordinateSet()
    
    c_flex_m = coord_set.get("hip_flexion_r") if coord_set.contains("hip_flexion_r") else None
    c_add_m = coord_set.get("hip_adduction_r") if coord_set.contains("hip_adduction_r") else None
    c_rot_m = coord_set.get("hip_rotation_r") if coord_set.contains("hip_rotation_r") else None
    c_rod = coord_set.get("knee_angle_r") if coord_set.contains("knee_angle_r") else None

    hilo_muslo = LectorSensor(PUERTO_MUSLO, "MUSLO", r_cal_muslo, q_init_muslo, datos, es_muslo=True)
    hilo_pantorrilla = LectorSensor(PUERTO_PANTORRILLA, "PANTORRILLA", r_cal_pantorrilla, q_init_pantorrilla, datos, es_muslo=False)
    
    hilo_muslo.start()
    hilo_pantorrilla.start()
    
    buffer_captura = {
        'time': [], 'hip_flex': [], 'hip_add': [], 'hip_rot': [], 'knee_flex': []
    }
    
    os.system('cls' if os.name == 'nt' else 'clear')
    print("\nIniciando Captura de Datos. Presione Ctrl+C para detener y exportar.\n")
    
    tiempo_inicio = time.perf_counter()
    
    try:
        while True:
            tiempo_actual = time.perf_counter() - tiempo_inicio
            
            r_muslo, alpha_muslo, a_scom_M, gyro_M = hilo_muslo.get_datos_cinematicos()
            r_pantorrilla, alpha_pantorrilla, a_scom_P, gyro_P = hilo_pantorrilla.get_datos_cinematicos()

            if r_muslo is not None and r_pantorrilla is not None:
                ang_muslo = r_muslo.as_euler('xyz', degrees=False)
                ang_pant = r_pantorrilla.as_euler('xyz', degrees=False)
                
                flexion_hip = -ang_muslo[1]   
                adduccion_hip = ang_muslo[2]  
                rotacion_hip = -ang_muslo[0]  
                flexion_knee = min(0.0, -ang_pant[1] - flexion_hip)
                
                buffer_captura['time'].append(tiempo_actual)
                buffer_captura['hip_flex'].append(flexion_hip)
                buffer_captura['hip_add'].append(adduccion_hip)
                buffer_captura['hip_rot'].append(rotacion_hip)
                buffer_captura['knee_flex'].append(flexion_knee)

                tau_euler_P = np.dot(I_p, alpha_pantorrilla) + np.cross(gyro_P, np.dot(I_p, gyro_P))
                F_rodilla = m_pant * a_scom_P 
                r_CoM_a_Rodilla_P = np.array([0, scom_pct_p * L_pant, 0])
                tau_rodilla = tau_euler_P + np.cross(r_CoM_a_Rodilla_P, F_rodilla)
                
                Rot_Relativa_P_a_M = r_muslo.inv() * r_pantorrilla
                F_rodilla_en_Muslo = Rot_Relativa_P_a_M.apply(F_rodilla)
                tau_rodilla_en_Muslo = Rot_Relativa_P_a_M.apply(tau_rodilla)
                
                tau_euler_M = np.dot(I_m, alpha_muslo) + np.cross(gyro_M, np.dot(I_m, gyro_M))
                F_cadera = (m_muslo * a_scom_M) + F_rodilla_en_Muslo
                r_CoM_a_Cadera_M = np.array([0, scom_pct_m * L_muslo, 0])
                r_CoM_a_Rodilla_M = np.array([0, -(1 - scom_pct_m) * L_muslo, 0]) 
                
                tau_cadera = tau_euler_M + np.cross(r_CoM_a_Cadera_M, F_cadera) - np.cross(r_CoM_a_Rodilla_M, F_rodilla_en_Muslo) + tau_rodilla_en_Muslo
                
                if c_flex_m: c_flex_m.setValue(estado, flexion_hip)
                if c_add_m: c_add_m.setValue(estado, adduccion_hip)
                if c_rot_m: c_rot_m.setValue(estado, rotacion_hip)
                if c_rod: c_rod.setValue(estado, flexion_knee)
                modelo.realizePosition(estado)
                modelo.getVisualizer().show(estado)
                
                str_hip = f"Hip [Flex: {np.degrees(flexion_hip):5.1f}°, Add: {np.degrees(adduccion_hip):5.1f}°, Rot: {np.degrees(rotacion_hip):5.1f}°] | Knee Flex: {np.degrees(flexion_knee):5.1f}°"
                str_am = f"[{alpha_muslo[0]:5.1f}, {alpha_muslo[1]:5.1f}, {alpha_muslo[2]:5.1f} ]"
                str_ap = f"[{alpha_pantorrilla[0]:5.1f}, {alpha_pantorrilla[1]:5.1f}, {alpha_pantorrilla[2]:5.1f} ]"
                str_alpha = f"α Muslo: {str_am} | α Pant: {str_ap} rad/s²"
                str_tau = f"Torque (Nm) -> CADERA: {tau_cadera[0]:6.2f} | RODILLA: {tau_rodilla[0]:6.2f} | Tiempo: {tiempo_actual:.2f}s"
                
                sys.stdout.write(f"{str_hip}\n{str_alpha}\n{str_tau}\033[F\033[F")
                sys.stdout.flush()
            
            time.sleep(0.015)

    except KeyboardInterrupt:
        print("\n\n\n\nFinalizando captura...")
    finally:
        hilo_muslo.detener()
        hilo_pantorrilla.detener()
        hilo_muslo.join(timeout=1.0)
        hilo_pantorrilla.join(timeout=1.0)
        
        if len(buffer_captura['time']) > 0:
            exportar_a_mot("captura_movimiento.mot", buffer_captura)
            print("""
\nEl modelo actual posee 92 músculos a través de los cuales se distribuyeron las fuerzas ejercidas por y sobre el cuerpo, de los cuales 46 corresponden a la parte derecha de la cadera y la pierna. Para facilitar la búsqueda de los mismos se sugiere buscarlos a través de la siguiente organización:
                
1. Flexores de cadera: iliacus_r, psoas_r, tfl_r y pect_r
2. Extensores de cadera: glut_max1_r, glut_max2_r, glut_max3_r, add_mag1_r, add_mag2_r y add_mag3_r
3. Estabilizadores laterales: glut_med1_r, glut_med2_r, glut_med3_r, glut_min1_r, glut_min2_r, glut_min3_r, add_long_r, add_brev_r, gem_r y peri_r
4. Biarticulares anteriores: rect_fem_r y sar_r
5. Biarticulares posteriores: semimem_r, semiten_r, bifemlh_r y grac_r 
6. Extensores de rodilla: vas_int_r, vas_med_r, vas_lat_r y quad_fem_r 
7. Flexor de rodilla: bifemsh_r
8. Flexores plantares: med_gas_r, lat_gas_r, soleus_r, tib_post_r, flex_dig_r, flex_hal_r, per_brev_r y per_long_r
9. Doxiflesores plantares: tib_ant_r, per_tert_r, ext_dig_r y ext_hal_r
10. Estabilizadores del tronco: ercspn_r, intobl_r, extobl_r
""")

if __name__ == "__main__":
    main()