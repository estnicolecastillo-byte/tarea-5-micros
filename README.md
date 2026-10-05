# Control de Brazo Robótico URDF en Tiempo Real con ESP32, Python y PyBullet

Este proyecto implementa la integración entre un sistema embebido (**ESP32**), comunicación serie **UART** y un entorno de simulación física 3D en **Python** con **PyBullet**, para manipular el modelo robótico definido en `brazo.urdf`.

---

## Descripción del Proyecto

El objetivo principal de esta práctica es integrar hardware embebido con simulación 3D. Se capturan lecturas analógicas mediante potenciómetros conectados a una tarjeta **ESP32**; los datos se empaquetan y se envían por puerto serie (**UART/USB**) hacia la PC, donde un script de **Python** los procesa en tiempo real para articular el modelo 3D definido en formato **URDF** (`brazo.urdf`).

El modelo del brazo (`robot_con_pinza`) está compuesto por:

- Una base cilíndrica fija (`base_link`).
- Dos articulaciones rotacionales (`joint_1` y `joint_2`).
- Una pinza prismática (`joint_gripper`) con dos dedos (`joint_dedo_izq` y `joint_dedo_der`).

---

## Arquitectura del Sistema

```text
┌───────────────────────────┐
│ Potenciómetros (Hardware) │
└─────────────┬─────────────┘
              │ (Señales analógicas 0 - 3.3 V)
              v
┌───────────────────────────┐
│        Placa ESP32        │ ---> Lee ADC de 12 bits (0 - 4095)
└─────────────┬─────────────┘
              │ (Trama CSV por UART / 115200 baudios)
              v
┌───────────────────────────┐
│ Script Python (pyserial)  │ ---> Parsea y mapea valores a radianes/metros
└─────────────┬─────────────┘
              │ (Comandos de posición)
              v
┌───────────────────────────┐
│  Motor de Simulación 3D   │ ---> Renderiza movimiento en tiempo real
│        (PyBullet)         │
└───────────────────────────┘
```

---

## Montaje del Circuito Físico

Se implementó un circuito en protoboard utilizando **3 potenciómetros** conectados a las entradas analógicas (ADC) de la ESP32:

| Potenciómetro | Pin ESP32 | Función |
|---------------|-----------|---------|
| Potenciómetro 1 | GPIO 34 | Controla la Articulación 1 (J1) |
| Potenciómetro 2 | GPIO 35 | Controla la Articulación 2 (J2) |
| Potenciómetro 3 | GPIO 32 | Controla la apertura y cierre de la Pinza (Gripper) |

Cada potenciómetro se alimenta con 3.3 V y GND de la ESP32, y su pin central (cursor) va al GPIO correspondiente.

![Montaje del circuito en protoboard](img/montaje_circuito.jpg)

---

## Cinemática y Mapeo Matemático

Cada entrada analógica de la ESP32 entrega un valor entero de 12 bits entre 0 y 4095. Para convertir estos datos crudos a las unidades requeridas por el archivo URDF (radianes para articulaciones rotacionales y metros para prismáticas) se utiliza la fórmula de interpolación lineal:

```text
y = y_min + ((x - x_min) / (x_max - x_min)) * (y_max - y_min)
```

### Tabla de Articulaciones y Rangos

| Articulación | Tipo de joint | Rango ADC ESP32 | Límites URDF | Rango PyBullet |
|--------------|---------------|-----------------|--------------|----------------|
| `joint_1` | Revolute | 0 – 4095 | -143° a 143° | -2.49 a 2.49 rad |
| `joint_2` | Revolute | 0 – 4095 | -115° a 115° | -2.00 a 2.00 rad |
| `joint_gripper` / Pinza | Prismatic | 0 – 4095 | 0.00 a 0.05 m | 0.00 a 0.05 m |

---

## Código Fuente

### 1. Firmware ESP32 (`esp32_control.ino`)

```cpp
// Definición de pines analógicos en la ESP32
const int PIN_J1 = 34;    // Potenciómetro para Articulación 1
const int PIN_J2 = 35;    // Potenciómetro para Articulación 2
const int PIN_PINZA = 32; // Potenciómetro para la Pinza

void setup() {
  // Inicialización de la comunicación serie a 115200 baudios
  Serial.begin(115200);
  pinMode(PIN_J1, INPUT);
  pinMode(PIN_J2, INPUT);
  pinMode(PIN_PINZA, INPUT);
}

void loop() {
  // Lectura del ADC de 12 bits (0 a 4095)
  int val_j1 = analogRead(PIN_J1);
  int val_j2 = analogRead(PIN_J2);
  int val_pinza = analogRead(PIN_PINZA);

  // Envío de la trama en formato CSV separado por comas
  Serial.print(val_j1);
  Serial.print(",");
  Serial.print(val_j2);
  Serial.print(",");
  Serial.println(val_pinza); // Salto de línea como delimitador de trama

  delay(20); // Tasa de refresco (~50 Hz)
}
```

### 2. Script de Control y Simulación (`control_brazo.py`)

```python
import serial
import pybullet as p
import pybullet_data
import time

# Configuración del puerto COM y velocidad UART
PUERTO_SERIAL = 'COM7'  # Ajustar según el puerto asignado a tu ESP32
BAUD_RATE = 115200

# 1. Inicialización del entorno gráfico PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# 2. Carga del modelo URDF del brazo
try:
    robot_id = p.loadURDF("brazo.urdf", [0, 0, 0], useFixedBase=True)
    print("Modelo URDF cargado exitosamente.")
except Exception as e:
    print(f"Error al cargar el archivo URDF: {e}")
    exit()

NUM_JOINTS = p.getNumJoints(robot_id)

def mapear(valor, in_min, in_max, out_min, out_max):
    """Función de interpolación lineal para mapear rangos."""
    return (valor - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# 3. Apertura del puerto serie UART
try:
    puerto = serial.Serial(PUERTO_SERIAL, BAUD_RATE, timeout=0.1)
    print(f"Conexión UART establecida en {PUERTO_SERIAL}.")
except Exception as e:
    print(f"Error abriendo puerto serie: {e}")
    exit()

# 4. Bucle principal de simulación y control
while p.isConnected():
    try:
        p.stepSimulation()
        time.sleep(0.01)

        if puerto.in_waiting > 0:
            linea = puerto.readline().decode('utf-8', errors='ignore').strip()
            if linea:
                datos = linea.split(',')
                if len(datos) == 3:
                    raw_j1, raw_j2, raw_pinza = map(int, datos)

                    # Conversión de valores analógicos a radianes y metros
                    rad_j1 = mapear(raw_j1, 0, 4095, -2.49, 2.49)
                    rad_j2 = mapear(raw_j2, 0, 4095, -2.00, 2.00)
                    m_pinza = mapear(raw_pinza, 0, 4095, 0.00, 0.05)

                    # Aplicar posiciones a los servomotores virtuales
                    if NUM_JOINTS > 0:
                        p.setJointMotorControl2(robot_id, 0, p.POSITION_CONTROL, targetPosition=rad_j1)
                    if NUM_JOINTS > 1:
                        p.setJointMotorControl2(robot_id, 1, p.POSITION_CONTROL, targetPosition=rad_j2)
                    if NUM_JOINTS > 2:
                        p.setJointMotorControl2(robot_id, 2, p.POSITION_CONTROL, targetPosition=m_pinza)
                    if NUM_JOINTS > 3:
                        p.setJointMotorControl2(robot_id, 3, p.POSITION_CONTROL, targetPosition=m_pinza)

                    print(f"J1: {rad_j1:.2f} rad | J2: {rad_j2:.2f} rad | Pinza: {m_pinza:.3f} m")

    except KeyboardInterrupt:
        print("Simulación finalizada por el usuario.")
        break
    except Exception:
        # Si llega un dato erróneo, no cierra el programa
        continue

puerto.close()
p.disconnect()
```

---

## Evidencias de Funcionamiento

### Montaje físico

![ESP32 y potenciómetros en protoboard](img/montaje_circuito.jpg)

### Modelo cargado en VS Code (URDF Preview) y terminal

![Proyecto en VS Code con URDF Preview](img/vscode_urdf_preview.png)

### Simulación en PyBullet

![Ventana de PyBullet con el brazo](img/pybullet_simulacion.png)



