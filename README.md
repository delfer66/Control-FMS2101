***
Este es el software base que se integró en el sistema de manufactura flexible o FMS 2101 repotenciado desde cero.
Desarrollado principalmente para comunicar cada módulo o estación y ser siempre seguro en su funcionamiento en la FMS, también permite la automatización de la fabricación de piezas mecanizadas, cuenta con la posibilidad de funcionamiento manual o por el usuario.
Este programa cuenta con más de 8000 líneas de código y fue desarrollado en 2 meses en lenguaje C para controlar un sistema de manufactura flexible que cuenta con:
    • 	Una estación de Fresado.
    • 	Una estación de Torneado.
    • 	Un riel de desplazamiento lineal.
    • 	Un controlador principal.
    • 	Un mando de control.
    • 	Capacidad de control de un robot manipulador.
    • 	Gabinete de interconexiones y potencia.
Este software se divide en 4 archivos para los microcontroladores de:
    •	La estación de Fresado (Arduino Due)
    •	La estación de Torneado (Arduino Due)
    •	Controlador principal (Arduino Mega)
    •	Mando de control (Arduino Nano)
El código es totalmente estable para el usuario siempre y cuando se siga al pie de la letra el preparatorio y laboratorio desarrollado para el funcionamiento de la estación, no puedo garantizar este desarrollo más allá de esto debido a la rapidez en su desarrollo y por supuesto sepan disculparme no haber documentado el funcionamiento completo.
Esto código está colocado en Github  como respaldo y para futuros desarrollos en el software para la estación FMS 2101.
En primer lugar recomiendo revisar mi trabajo de titulación [REPOTENCIACIÓN Y MODERNIZACIÓN DE LA ESTACIÓN DE MANUFACTURA FMS 2101 DEL DEPARTAMENTO DE ELÉCTRICA, ELECTRÓNICA Y TELECOMUNICACIONES DE LA UNIVERSIDAD DE LAS FUERZAS ARMADAS ESPE](http://repositorio.espe.edu.ec/xmlui/handle/21000/21678), para mayor información y detalles de todo el proyecto.
Además revisar la [Guía preparatoria de la estación FMS 2101.pdf](https://github.com/delfer66/Mando-FMS/blob/master/Guía_preparatoria_de_la_estación_FMS_2101.pdf) y la [Guía de laboratorio de la estación FMS 2101.pdf](https://github.com/delfer66/Mando-FMS/blob/master/Guía_de_laboratorio_de_la_estación_FMS_2101.pdf). Para conocer el funcionamiento básico del software.

***
# ¿Qué es la estación o sistema de manufactura flexible o FMS 2101?

Este software fue creado específicamente para controlar esta estación FMS2101 que se muestra a continuación:
![alt tag](https://github.com/delfer66/Mando-FMS/blob/master/res/fms.png)
Esta estación permite introducirse al estudio de la manufactura flexible que es uno de los pilares de la industria 4.0.

***
# Características del software

Este software cuenta con las siguientes características
    •	Control y automatización completa de la estación de manufactura flexible o FMS.
    •	Capacidad de comunicación UART utilizando la interface RS232.
    •	Seguridad y robustez en la comunicación utilizando puertos E/S que aseguran la comunicación.
    •	Control y automatización de la estación de Fresado y Torneado.
    •	Capacidad de Mecanizado (Fresado y Torneado) manual utilizando el mando de control.
    •	Capacidad de Mecanizado (Fresado y Torneado) Automático utilizando el hardware de Arduino Uno con el software de RGBL.
    •	Conmutación entre el mecanizado automático y manual, seleccionando RGBL o el mando de control.
    •	Control del riel de desplazamiento lineal utilizando el mando de control.
    •	Control de un manipulador robótico utilizando el mando de control.

***
# Funcionamiento del Software:

El programa tiene el propósito de controlar la estación mediante comunicación UART, donde la NCL estación de torneado, NCM estación de fresado y el mando se conectan directamente al control principal cada uno con una comunicación UART dedicada,  Adicionalmente la comunicación también se realiza mediante entradas y salidas o (E/S) digitales permitiendo el control directo, seguro y estable con los motor reductores DC de la estación de torneado, fresado, riel de desplazamiento lineal y robot manipulador. Para el control del riel de desplazamiento lineal y el robot manipulador únicamente se utilizan E/S digitales y para el resto incorpora las dos maneras de comunicación.
En el siguiente grafico se observa la conexión que existe entre las diferentes estaciones.
 ![alt tag](https://github.com/delfer66/Mando-FMS/blob/master/res/fms_block.png)
La comunicación UART permite ordenar a la estación las acciones que debe tomar como por ejemplo que el usuario controle la máquina de fresado, torneado, riel o control principal, también permitir que el control se realice con el software RGBL.
Instalación del software:
    •	Desconecte la energía a la estación FMS.
    •	Desconecte el microcontrolador que va a programas de la PCB.
    •	Conecte el microcontrolador a la computadora.
    •	Cargue el software en el IDE de Arduino a cada microcontrolador como se muestra a continuación.
    •	El programa ControlPrincipalFMS2101.ino para el microcontrolador Arduino Mega, que se encuentra dentro de la caja.
![alt tag](https://github.com/delfer66/Mando-FMS/blob/master/res/cp.png)
        o	Al abrir la caja
![alt tag](https://github.com/delfer66/Mando-FMS/blob/master/res/cp1.png)
    •	El programa MandoFMS2101.ino en el microcontrolador Arduino Nano que se encuentra dentro del mando.
![alt tag](https://github.com/delfer66/Mando-FMS/blob/master/res/mando.png)
        o	Al abrir la caja
![alt tag](https://github.com/delfer66/Mando-FMS/blob/master/res/mando1.png)
    •	El programa TornoNCL2000.ino en el microcontrolador Arduino Due.
![alt tag](https://github.com/delfer66/Mando-FMS/blob/master/res/torno.png)
    •	El programa FresadoraNCM2000.ino en el microcontrolador Arduino Due.
![alt tag](https://github.com/delfer66/Mando-FMS/blob/master/res/fresadora.png)

***
# Expectativas a futuro:

Debido a que el hardware implementado es mucho más potente de lo requerido para la estación y que la adición de más componentes es relativamente sencilla, pero debido a las limitaciones de tiempo en el proyecto no se logró implementar las siguientes características que llevarían la estación a la industria 4.0.
    •	Desarrollo de software de automatización y manufactura flexible enfocado en la producción en serie.
    •	Repotenciación, fabricación o integración de un manipulador robótico exclusivo para la estación.
    •	Integración de medios de comunicación como: Ethernet, ModBUS, Wi-fi o Bluetooth.

```xml
<manifest xmlns:android="http://schemas.android.com/apk/res/android"
    package="com.tutsplus.matt.bluetoothscanner" >
     
    <uses-permission android:name="android.permission.BLUETOOTH" />
    <uses-permission android:name="android.permission.BLUETOOTH_ADMIN" />
```