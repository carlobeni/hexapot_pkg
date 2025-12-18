## Tutorial de conexion remota a la pi5 por SSH 
Habiendo encendido la pi5 hacegurandose de que no solicita autenticacion, se puede conectar a la pi5 en Xor desde la PC
### 1. Habilitar terminal por SSH
```bash
ssh carlobeni@192.168.0.30
```
password: 2357
### 2. Correr launcher de ros2 en la pi
```bash
ros2 launch articubot_pi_sensor sensor_launch.py
```
### 3. Visualizar Ubuntu completo de la pi (opciona)
- Activar Servicio VNC (En la pi5)
```bash
x11vnc -display :0 -forever -shared -rfbauth /home/carlobeni/.vnc/passwd
```
- Intalar remmina (En la PC)
```bash
sudo apt install remmina
```
- Iniciar sesion remota con remmina con la siguiente configuracion

Protocol: Remina VNC Plugin

Server: 192.168.0.30:5900

User password: 2357



