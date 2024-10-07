import serial
import uinput
import time

ser = serial.Serial('/dev/ttyACM0', 115200)

# Cria um novo dispositivo de mouse
device = uinput.Device([
    uinput.BTN_LEFT,
    uinput.REL_X,
    uinput.REL_Y,
])

try:
    while True:
        # Espera pelo sinal de clique ou dados de movimento
        print('Waiting for data...')
        data = ser.readline().strip()
        
        try:
            data_str = data.decode('utf-8')
            print(f"Decoded data: {data_str}")
            
            if data_str.startswith('X:'):
                # Parseia os dados de movimento horizontal
                move_x = int(data_str.split(':')[1])
                device.emit(uinput.REL_X, move_x)
                print(f"Moved X by {move_x}")
                
            elif data_str.startswith('Y:'):
                # Parseia os dados de movimento vertical
                move_y = int(data_str.split(':')[1])
                device.emit(uinput.REL_Y, move_y)
                print(f"Moved Y by {move_y}")
            
            elif data_str.startswith('C:'):
                # Se o sinal de clique for recebido
                print("Mouse click detected!")
                device.emit(uinput.BTN_LEFT, 1)
                time.sleep(0.1)
                device.emit(uinput.BTN_LEFT, 0)
                print("Mouse click emitted")
        
        except UnicodeDecodeError:
            print(f"Received non-UTF-8 data: {data}")
        except ValueError:
            print(f"Error parsing data: {data_str}")

except KeyboardInterrupt:
    print("Program terminated by user")
except Exception as e:
    print(f"An error occurred: {e}")
finally:
    ser.close()