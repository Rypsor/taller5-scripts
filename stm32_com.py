import serial
import time

# --- CONFIGURACIÓN SERIAL ---
PORT_NAME = '/dev/ttyACM0' 
BAUD_RATE = 115200

# Lista de comandos simplificados
COMMANDS = [
    "led rojo", # Rojo
    "led verde", # Verde
    "led azul", # Azul
    "o"  # Apagar
]

def send_command(ser, command):
    """
    Envía un comando de un solo carácter. 
    Ya no espera respuesta, solo envía el carácter.
    """
    
    # 1. Limpiar buffers (buena práctica)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    
    # 2. El comando es solo el carácter
    command_to_send = command 
    
    print(f"-> Enviando: '{command}'")
    
    try:
        # Envía el comando (codificado a bytes)
        ser.write(command_to_send.encode('utf-8'))
        
        # 3. Pausa breve para que el STM32 procese la instrucción y cambie el LED
        time.sleep(0.1) 
        
    except Exception as e:
        print(f"ERROR: Ocurrió un error de comunicación: {e}")

if __name__ == "__main__":
    
    # Inicializa el puerto serial
    try:
        # NOTA: Eliminamos el timeout porque ya no usamos ser.readline()
        ser = serial.Serial(
            PORT_NAME, 
            BAUD_RATE 
        )
        time.sleep(2) # Espera a que el puerto serial se inicialice completamente
        print(f"*** Puerto Serial {PORT_NAME} abierto a {BAUD_RATE} bps ***")
        
        # Bucle de prueba
        for cmd in COMMANDS:
            send_command(ser, cmd)
            time.sleep(1) # Pausa más larga entre pruebas visuales
            
        print("*** Pruebas finalizadas. ***")
        ser.close()
        
    except serial.SerialException as e:
        print(f"ERROR: No se pudo abrir el puerto serial {PORT_NAME}.")
        print(f"Asegúrate de que el puerto sea correcto y no esté siendo usado: {e}")