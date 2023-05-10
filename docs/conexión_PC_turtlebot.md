# conexión turtlebot - PC
## Adaptador wifi PC TP-Link Archer T3U Plus

Instalación de drivers y puesta en marcha:
```
sudo apt update
sudo apt install git linux-headers-generic dkms dwarves
cp /sys/kernel/btf/vmlinux /usr/lib/modules/`uname -r`/build/
git clone https://github.com/RinCat/RTL88x2BU-Linux-Driver
cd RTL88x2BU-Linux-Driver
sudo make uninstall
make clean
make
sudo make install
sudo modprobe 88x2bu
sudo reboot
```

Comprobar que aparece el nuevo adaptador con `ip a`

Activar modo AP en el adaptador WIFI:
1. Configuración de wifi
2. ⋮ Activar punto de acceso inalámbrico...
3. Activar
4. SSID: turtlebot4PC
5. Contraseña: 12345678

## Conexión turtlebot al AP

Mediante conexión Ethernet, configurar el PC para que tenga IP estática:
1. IP: 192.168.185.5
2. mask: 255.255.255.0
3. gateway: 192.168.185.1

Conectarse mediante ssh al turtlebot `turtlebot@192.168.185.3` (contraseña: turtlebot4) e introducir el comando para configurar la conexión wifi: `turtlebot4-setup`

Una vez dentro seleccionar Wi-Fi Setup con la siguiente configuración:

<img src="imgs/wifisetup.png">

Despues guardar y aplicar cambios, se reiniciará y ya tendrá IP.

## Activar Discovery-Server

Al usar ROS2 Humble es recomendable activar el discovery-server para que solamente se conecte la Raspberry al AP, y que el Create 3 actúe como un cliente local. De esta manera nos ahorramos problemas con el multicasting. 

Para activar el Discovery-Server se siguen las instrucciones del [manual de usuario](https://turtlebot.github.io/turtlebot4-user-manual/setup/discovery_server.html).

Al final de todos los pasos deberemos poder ver todos los topics de la raspberry y del Create 3 desde el PC.

## Configurar chrony en el PC

Editar el archivo `/etc/chrony/chrony.conf` y añadir lo siguiente:

```
# cnfig ntp sync with turtlebot
server 10.42.0.21 presend 0 minpoll 0 maxpoll 0 iburst  prefer trust
# enable serving time to ntp clients on 10.42.0.0 subnet
allow 10.42.0.0/24
# serve time even offline
local stratum 10
```

Tendremos que abrir el puerto 123 en el protocolo UDP para aceptar peticiones NTP:

```Bash
sudo ufw allow from any to any port 123 proto udp
```

Y en la Raspberry editar `/etc/chrony/chrony.conf` y añadir: 

```
server 10.42.0.1 iburst prefer
```

De esta forma el PC actuará como servidor NTP.

