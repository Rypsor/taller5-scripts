import serial
import time
import sys

# --- CONFIGURACIÓN SERIAL ---
PORT_NAME = '/dev/ttyACM0' 
BAUD_RATE = 115200

def send_command(ser, command, wait_time=0.1, get_response=False):
    """
    Envía un comando al microcontrolador a través del puerto serie.

    Args:
        ser (serial.Serial): El objeto del puerto serie.
        command (str): El comando a enviar (sin el carácter de retorno de carro).
        wait_time (float): Tiempo de espera en segundos después de enviar el comando.
        get_response (bool): Si es True, lee y devuelve una línea de respuesta.

    Returns:
        str: La respuesta del dispositivo si get_response es True, si no, None.
    """
    full_command = command + '\r'
    print(f"-> Enviando: '{command}'")
    
    try:
        ser.reset_input_buffer()
        ser.write(full_command.encode('utf-8'))
        time.sleep(wait_time)

        if get_response:
            response = ser.readline().decode('utf-8').strip()
            print(f"<- Recibido: '{response}'")
            return response

    except Exception as e:
        print(f"ERROR: Ocurrió un error de comunicación: {e}")
        return None

def run_analysis(ser, command, filename):
    """
    Ejecuta un comando de análisis en el STM32 y guarda los datos CSV recibidos.

    Args:
        ser (serial.Serial): El objeto del puerto serie.
        command (str): El comando de análisis a ejecutar (ej. "analisis_ic_vb").
        filename (str): El nombre del archivo CSV donde se guardarán los datos.
    """
    print(f"\n--- Iniciando Análisis: {command} ---")
    print(f"Los datos se guardarán en '{filename}'")
    
    full_command = command + '\r'
    
    try:
        ser.reset_input_buffer()
        ser.write(full_command.encode('utf-8'))
        
        # Aumentar el timeout para dar tiempo a que se complete el análisis
        ser.timeout = 20 # 20 segundos

        lines_received = 0
        with open(filename, 'w') as f:
            # Añadir encabezados al CSV
            if command == "analisis_ic_vb":
                f.write("vb_adc,ve_adc\n")
            elif command == "analisis_ic_vc":
                f.write("duty_vc,ve_adc\n")

            while True:
                response = ser.readline().decode('utf-8').strip()
                if response:
                    f.write(response + '\n')
                    lines_received += 1
                else:
                    # Si no se recibe nada, el análisis ha terminado
                    break

        print(f"Análisis completado. Se recibieron {lines_received} líneas de datos.")
        
    except Exception as e:
        print(f"ERROR durante el análisis: {e}")
    finally:
        # Restaurar el timeout original
        ser.timeout = 1

if __name__ == "__main__":
    try:
        ser = serial.Serial(PORT_NAME, BAUD_RATE, timeout=1)
        time.sleep(2)
        print(f"*** Puerto Serial {PORT_NAME} abierto a {BAUD_RATE} bps ***")
        
        # --- Pruebas de Comandos ---
        print("\n--- Probando comandos LED ---")
        send_command(ser, "led rojo")
        send_command(ser, "led verde")
        send_command(ser, "led azul")
        send_command(ser, "apagar led")

        print("\n--- Probando comandos de control y lectura de Tarea 3 ---")
        send_command(ser, "set_vc 512")
        send_command(ser, "set_vb 400")

        send_command(ser, "get_ve", get_response=True)
        send_command(ser, "get_vb", get_response=True)

        # --- Ejecución de Análisis ---
        run_analysis(ser, "analisis_ic_vb", "analisis_ic_vs_vb.csv")
        run_analysis(ser, "analisis_ic_vc", "analisis_ic_vs_vc.csv")

        print("\n*** Todas las pruebas y análisis han finalizado. ***")
        
    except serial.SerialException as e:
        print(f"ERROR: No se pudo abrir el puerto serial {PORT_NAME}.")
        print(f"Asegúrate de que el dispositivo esté conectado y el puerto sea correcto: {e}")
        sys.exit(1)
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Puerto serial cerrado.")