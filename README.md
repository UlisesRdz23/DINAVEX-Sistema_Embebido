DINAVEX – Sistema embebido
Repositorio del código final del sistema embebido del robot autoequilibrado DINAVEX, desarrollado como proyecto integrador de Mecatrónica en la Facultad de Ingeniería de la Universidad Autónoma de San Luis Potosí (UASLP).

Descripción:

El sistema embebido está basado en un microcontrolador STM32F446RE y se encarga de ejecutar las tareas de adquisición de información y control necesarias para la estabilización del robot.

El programa desarrollado contempla las siguientes funciones:
- Adquisición y procesamiento de datos de la unidad de medición inercial (IMU).
- Lectura de la velocidad de las ruedas.
- Ejecución del algoritmo de control de estabilización.
- Envío de las señales de control a los controladores de los motores.
- Transmisión de datos hacia la Raspberry Pi para su visualización en la interfaz del sistema.
Hardware principal
- Tarjeta de desarrollo STM32 Nucleo-64 con microcontrolador STM32F446RE.
- Unidad de medición inercial MPU-6050 (módulo GY-521).
- Motores BLDC y sus controladores.
- Raspberry Pi 400 para recepción y visualización de datos.
  
Entorno de desarrollo
El proyecto fue desarrollado en STM32CubeIDE, utilizando el lenguaje C.

Estructura del repositorio

Archivo o carpeta	Descripción

Inc/	Archivos de cabecera del programa.

Src/	Archivos fuente y lógica del sistema embebido.

Startup/	Archivos de arranque del microcontrolador.

.project y .cproject	Configuración del proyecto en STM32CubeIDE.

STM32F446RETX_FLASH.ld y STM32F446RETX_RAM.ld	Archivos de configuración de memoria del microcontrolador.


Cómo abrir el proyecto
1. Descargar el repositorio mediante Code → Download ZIP y descomprimirlo.
2. Abrir STM32CubeIDE.
3. Seleccionar File → Import → General → Existing Projects into Workspace.
4. Elegir la carpeta descomprimida que contiene los archivos .project y .cproject.
5. Importar el proyecto y verificar la configuración antes de compilarlo y programar la tarjeta STM32.
Nota: Para compilar y ejecutar el programa, deben estar disponibles las dependencias del proyecto y las conexiones de hardware correspondientes. Este repositorio documenta el código final del sistema embebido; no constituye por sí solo una guía completa de instalación del robot.

Proyecto DINAVEX
DINAVEX es una plataforma didáctica y experimental de dos ruedas con autoequilibrio, orientada al estudio e implementación de estrategias de control.

Uso académico. La operación del prototipo debe realizarse bajo supervisión y con las medidas de seguridad correspondientes.
