# Descripción general

Conjunto de programas que hacen posible cuantificiar la fuerza que un sujeto pone a los movimientos de una de sus piernas a través de analizar la cinematica y dinamica de la misma mediante sensores inerciales proporcionados por la facultad de inteligencia artificial.

# Descripción de componentes

## esp32_firmware

Código que se encarga de enviar los datos a través de comunicación Blueetooth. Durante los primeros 7 segundos desde su encendido el código se encarga de calibrar el giroscopio del dispositivo debido a errores de fábrica / bias, comúnes en sensores inerciales. Una vez que el sistema ha sido calibrado el dispositivo envía una cadena de 10 datos continuamente. Dicha cadena es la siguiente, donde tiempoActual hace referencia a un timestamp en segundos desde que el dispositivo fue encendido, los valores derivados de accel. describen la inclinación respecto a la gravedad, los valores gx, gy y gz_corregidos describen los giros del dispositivo, y el magnetómetro sirve para corregir la orientación respecto al norte magnético:

```arduino

String paquete = String(tiempoActual) + "," + 
                     String(accel.acceleration.x, 3) + "," + 
                     String(accel.acceleration.y, 3) + "," + 
                     String(accel.acceleration.z, 3) + "," + 
                     String(gx_corregido * 57.2958, 2) + "," +
                     String(gy_corregido * 57.2958, 2) + "," + 
                     String(gz_corregido * 57.2958, 2) + "," + 
                     String(mag.magnetic.x, 2) + "," + 
                     String(mag.magnetic.y, 2) + "," + 
                     String(mag.magnetic.z, 2);

```

## opensim_tools

### geometry

Paquetes y recursos tridimensionales necesarios para que las herramientas/extensiones de OpenSim puedan funcionar en tiempo real

### GaitModel.osim

Modelo tridimensional articulado en el que se reflejará el movimiento de la pierna. 

## python_simulation

El script principal (opensim_kinematics_streamer.py) implementa una arquitectura multihilo para recibir y procesar el flujo de datos Bluetooth a 100 Hz de ambos sensores en paralelo. Aplica el filtro de fusión sensorial de Madgwick para transformar las lecturas en cuaterniones absolutos de orientación tridimensional. Durante el arranque, ejecuta una fase de calibración estática para calcular la matriz inversa de referencia (efecto "tara"), conservando la memoria espacial del filtro para evitar saltos o desfases iniciales. Posteriormente, descompone las rotaciones relativas en ángulos de Euler para controlar la flexión, aducción y rotación de la cadera, mientras aísla el ángulo de la rodilla restando la inclinación del muslo a la de la pantorrilla. Todos estos valores se inyectan en tiempo real a la API de OpenSim para actualizar la postura del modelo tridimensional.

> Instrucciones de uso: Para que el script funcione de manera adecuada se hacen las siguientes recomendaciones:

> 1. Haber encendido y calibrado el dispositivo como se describió anteriormente.
> 2. Instrumentar al sujeto una vez que los dispositivos hayan sido calibrados, lo más centrados sobre la rodilla y el muslo que se pueda y de manera que no se >    resbalen durante su uso.
> 3. Al momento de ejecutar el script es ideal que el usuario se mantenga en una postura erguida y con las piernas en una posición rígida durante 2 minutos, el >    tiempo de calibración, para evitar problemas de desviaciones. 
> 4. Es altamente recomendable mantener conectado su equipo en caso de que sea portátil, puesto que estos tienden a bajar su frecuencia de funcionamiento al >    momento de desconectarse. Se sugiere una frecuencia de procesamiento igual o mayor a los 2.9 GHz. 

## environment.yml / requirements.txt

La presencia de dos archivos de configuración responde a la necesidad de garantizar la reproducibilidad del proyecto en diferentes entornos de desarrollo. El archivo environment.yml está optimizado para Anaconda o Miniconda, siendo la vía recomendada para gestionar correctamente las dependencias complejas y los binarios compilados en C/C++ que requiere la API de OpenSim. Por otro lado, requirements.txt se incluye como una alternativa ligera e híper compatible para desarrolladores que prefieren trabajar con entornos virtuales puros de Python (venv y pip) sin depender del ecosistema de Conda.

![Python](https://img.shields.io/badge/python-3.10%2B-3776AB?style=for-the-badge&logo=python&logoColor=white)
![ESP32](https://img.shields.io/badge/ESP32-ICM20948-E7352C?style=for-the-badge&logo=espressif&logoColor=white)
![OpenSim](https://img.shields.io/badge/OpenSim-Biomechanics-008080?style=for-the-badge)