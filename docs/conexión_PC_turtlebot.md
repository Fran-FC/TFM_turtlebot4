# conexión turtlebot - PC
## Adaptador wifi PC TP-Link Archer T3U Plus

Instalación de drivers y puesta en marcha:
```Bash
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

### Habilitar redireccionamiento NAT

Añadir una regla iptables para habilitar el tráfico a modo de router NAT:

```Bash
sudo iptables -t nat -s 10.42.0.0/24 -A POSTROUTING -j MASQUERADE
```

Guardar la regla de manera persistente:

```Bash
sudo su
apt install iptables-persistent
iptables-save > /etc/iptables/rules.v4
```

## Conexión turtlebot al AP

Mediante conexión Ethernet, configurar el PC para que tenga IP estática:
1. IP: 192.168.185.5
2. mask: 255.255.255.0
3. gateway: 192.168.185.1

Conectarse mediante ssh al turtlebot `turtlebot@192.168.185.3` (contraseña: turtlebot4) e introducir el comando para configurar la conexión wifi: `turtlebot4-setup`

Una vez dentro seleccionar Wi-Fi Setup con la siguiente configuración:

<img src="imgs/wifisetup.png">

Despues guardar y aplicar cambios, se reiniciará y ya tendrá IP.

### Configurar DNS y gateway para acceso a internet

Para tener acceso a internet deberemos editar el archivo `/etc/netplan/50-wifis.yaml` para que tenga el siguiente aspecto:

```Yaml
network:
    version: 2
    wifis:
        renderer: NetworkManager
        wlan0:
            access-points:
                turtlebot4PC:
                    band: 5GHz
                    password: '12345678'
            addresses:
            - 10.42.0.21/24
            routes:
              - to: default
                via: 10.42.0.1
            dhcp4: false
            optional: true
```

Guardamos la configuración ejecutando `sudo netplan apply`. Con esto usaremos la dirección 10.42.0.1 como gateway de todo nuestro tráfico.


Falta añadir los servidores DNS editando el fichero `/etc/resolv.conf` y añadiendo el PC como servidor DNS:

```Bash
nameserver 10.42.0.1
```

Ahora actualizaremos el sistema e instalaremos un paquete para poder aplicar la configuración del DNS de forma permanente:

```Bash
sudo apt update; sudo apt upgrade -y; sudo apt install resolvconf -y
sudo systemctl enable resolvconf.service
sudo systemctl start resolvconf.service
```

Añadiremos la siguiente linea a `/etc/resolvconf/resolv.conf.d/head`: 

```Bash
nameserver 10.42.0.1
```

Reiniciamos y aplicamos los cambios: 

```BAsh
sudo resolvconf --enable-updates
sudo resolvconf -u
sudo systemctl restart resolvconf.service
sudo systemctl restart systemd-resolved.service
```

## Activar Discovery-Server

Al usar ROS2 Humble es recomendable activar el discovery-server para que solamente se conecte la Raspberry al AP, y que el Create 3 actúe como un cliente local. De esta manera nos ahorramos problemas con el multicasting. 

Para activar el Discovery-Server se siguen las instrucciones del [manual de usuario](https://turtlebot.github.io/turtlebot4-user-manual/setup/discovery_server.html).

Al final de todos los pasos deberemos poder ver todos los topics de la raspberry y del Create 3 desde el PC.

## Configurar chrony en el PC

Editar el archivo `/etc/chrony/chrony.conf` y añadir lo siguiente:

```Bash
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

```Bash
server 10.42.0.1 iburst prefer
```

De esta forma el PC actuará como servidor NTP.

## ERROR DE SINCRONIZACIÓN

El Create 3 tiene configurada la zona horaria en UTC, mientra que la zona horaria en Valencia es CEST (UTC +2), por lo que está 2 horas con retraso. De momento no se consigue configurar para que vaya correctamente.
