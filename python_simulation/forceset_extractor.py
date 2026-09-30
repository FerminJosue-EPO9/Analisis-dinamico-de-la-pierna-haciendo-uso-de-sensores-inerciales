import opensim as osim
import math

def calcular_distancia(v1, v2):
    """Calcula la distancia euclidiana entre dos objetos Vec3 de OpenSim."""
    dx = v2.get(0) - v1.get(0)
    dy = v2.get(1) - v1.get(1)
    dz = v2.get(2) - v1.get(2)
    return math.sqrt(dx**2 + dy**2 + dz**2)

def extraer_datos_del_modelo(ruta_modelo):
    print(f"Cargando modelo desde: {ruta_modelo}...")
    try:
        modelo = osim.Model(ruta_modelo)
        estado = modelo.initSystem()

        # --- LISTADO DE MÚSCULOS ---
        musculos = modelo.getMuscles()
        total_musculos = musculos.getSize()
        print(f"\nSe encontraron {total_musculos} músculos en el modelo.")
        print("-" * 40)

        lista_nombres = []
        for i in range(total_musculos):
            nombre_musculo = musculos.get(i).getName()
            lista_nombres.append(nombre_musculo)
            print(f"{i + 1}. {nombre_musculo}")
        print("-" * 40)

        # --- POSE NEUTRA (todas las coordenadas a 0) ---
        for i in range(modelo.getCoordinateSet().getSize()):
            modelo.getCoordinateSet().get(i).setValue(estado, 0.0)
        modelo.realizePosition(estado)

        # --- POSICIONES DE LOS CUERPOS EN GROUND ---
        femur_r_pos = modelo.getBodySet().get("femur_r").getPositionInGround(estado)
        femur_l_pos = modelo.getBodySet().get("femur_l").getPositionInGround(estado)
        tibia_r_pos = modelo.getBodySet().get("tibia_r").getPositionInGround(estado)
        talus_r_pos = modelo.getBodySet().get("talus_r").getPositionInGround(estado)
        calcn_r_pos = modelo.getBodySet().get("calcn_r").getPositionInGround(estado)
        toes_r_pos  = modelo.getBodySet().get("toes_r").getPositionInGround(estado)
        pelvis_pos  = modelo.getBodySet().get("pelvis").getPositionInGround(estado)

        # --- CÁLCULO DE LONGITUDES ---
        long_femur   = calcular_distancia(femur_r_pos, tibia_r_pos)
        long_tibia   = calcular_distancia(tibia_r_pos, talus_r_pos)
        long_pie     = calcular_distancia(calcn_r_pos, toes_r_pos)
        ancho_pelvis = calcular_distancia(femur_r_pos, femur_l_pos)

        # Torso: distancia desde origen de pelvis hasta el CoM del torso (en ground)
        torso_body = modelo.getBodySet().get("torso")
        torso_mc_local = torso_body.getMassCenter()
        torso_mc_ground = torso_body.findStationLocationInGround(estado, torso_mc_local)
        long_torso = calcular_distancia(pelvis_pos, torso_mc_ground)

        print("\n--- LONGITUDES DE SEGMENTOS (pose neutra) ---")
        print(f"Fémur  (cadera → rodilla):        {long_femur:.4f} m")
        print(f"Tibia  (rodilla → tobillo):       {long_tibia:.4f} m")
        print(f"Pie    (calcn → toes, proxy):     {long_pie:.4f} m")
        print(f"Pelvis (cadera_r → cadera_l):     {ancho_pelvis:.4f} m")
        print(f"Torso  (pelvis → CoM torso):      {long_torso:.4f} m")
        print("\n"+"-" * 40)

        return lista_nombres

    except Exception as e:
        print(f"Error al cargar el modelo: {e}")

if __name__ == "__main__":
    ruta = "../opensim_tools/modelo_sujeto_escalado.osim" #GaitModel.osim / modelo_sujeto_escalado.osim
    nombres_extraidos = extraer_datos_del_modelo(ruta)