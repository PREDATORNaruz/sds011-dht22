


This Programm reads data from SDS011 and DHT22 Sensors , it will log the data in a csv file every 3 Minutes .

Please make sure to change your own File path ... 

if not just do these commands on your Pi 

sudo mkdir /home/pi/Documents/embedded_sys_2

gcc sensors.c -o sens 

./sens to launch it 
the csv file will be found there too . . .


For the DHT22 sensor the data pin is attached to pin 3 on the WiringPi , Gpio22 on the pi 
SDS011 , works with usb , check if it has the same tty as the code to do so  just type sudo dmesg 
if its not /dev/ttyUSB0 , change the path to the one you found on the terminal .
