import opensim as osim

def extraer_musculos_del_modelo(ruta_modelo):
    print(f"Cargando modelo desde: {ruta_modelo}...")
    try:
        modelo = osim.Model(ruta_modelo)
        
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
        return lista_nombres

    except Exception as e:
        print(f"Error al cargar el modelo: {e}")

if __name__ == "__main__":
    
    ruta = "../opensim_tools/GaitModel.osim" 
    nombres_extraidos = extraer_musculos_del_modelo(ruta)