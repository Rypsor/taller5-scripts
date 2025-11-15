import serial
import time
import sys

# --- Configuración ---
# ¡CAMBIA ESTO! Usa el nombre de tu puerto.
# Linux:   '/dev/ttyACM0'
# Windows: 'COM3'
# macOS:   '/dev/tty.usbmodemXXXX'
PUERTO_SERIAL = '/dev/ttyACM0' 
BAUD_RATE = 115200
# ---------------------

def main():
    print(f"Intentando conectar a {PUERTO_SERIAL} a {BAUD_RATE} baudios...")

    try:
        # Abre el puerto serial. 
        # timeout=1 es opcional, pero bueno para no bloquear.
        ser = serial.Serial(PUERTO_SERIAL, BAUD_RATE, timeout=1)
        print(f"¡Conexión exitosa a {ser.name}!")
        print("-----------------------------------------------")
        print("Escribe los comandos y presiona 'Enter':")
        print("  'a' -> Levogiro (Izquierda)")
        print("  'd' -> Dextrogiro (Derecha)")
        print("  'w' -> Subir Velocidad")
        print("  's' -> Bajar Velocidad")
        print("  'f' -> Frenar")
        print("  'q' -> Salir del script")
        print("-----------------------------------------------")

        while True:
            # Pide al usuario que escriba un comando
            comando = input("Comando > ")

            if not comando:
                continue

            # Lee la primera letra del comando (ignora el 'Enter')
            letra = comando[0]

            # Comprueba si queremos salir
            if letra == 'q':
                print("Saliendo del script.")
                break
            
            # Comprueba si es un comando válido para el motor
            if letra in ['a', 'd', 'w', 's', 'f']:
                # ¡IMPORTANTE! 
                # Enviamos el comando como 'bytes'
                ser.write(letra.encode('utf-8'))
                print(f"Enviado: '{letra}'")
            else:
                print(f"'{letra}' no es un comando válido. (Prueba: a,d,w,s,f)")

    except serial.SerialException as e:
        print(f"Error: No se pudo abrir el puerto {PUERTO_SERIAL}.")
        print(f"Detalle: {e}")
        print("Asegúrate de:")
        print("  1. La placa esté conectada.")
        print(f"  2. El nombre del puerto ('{PUERTO_SERIAL}') sea correcto.")
        print("  3. (En Linux) Tengas permisos (ej. 'sudo chmod 666 /dev/ttyACM0' o estar en el grupo 'dialout').")
    except KeyboardInterrupt:
        print("\nSaliendo por Ctrl+C.")
    finally:
        # Asegúrate de cerrar el puerto al salir
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print(f"Puerto {PUERTO_SERIAL} cerrado.")

if __name__ == "__main__":
    main()