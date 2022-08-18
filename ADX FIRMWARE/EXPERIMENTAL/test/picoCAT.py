#*==*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
#* picoCAT.py
#* Testing bridge to test the connectivity between WSJT-X and PDX
#*
#* This program requires a virtual cable program such as com0com (Win) or socat (Linux) to operate
#*==*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
import time
import serial
from datetime import datetime

wsjtPort='COM13'
wsjtBaud=19200

pdxPort='COM5'
pdxBaud=19200

log=True


# configure the serial connections (the parameters differs on the device you are connecting to)

print("picoCAT.py")
print("Serial port bridge between WSJT-X and PDX for testing and debugging purposes")
print("WSJT-X("+wsjtPort+","+str(wsjtBaud)+") <<-->> PDX("+pdxPort+","+str(pdxBaud)+")")

#*----------------------------------------*
#* Initialize PDX end                     *
#*----------------------------------------*
ser = serial.Serial(
      port=pdxPort,
      baudrate=pdxBaud,
      parity=serial.PARITY_NONE,
      stopbits=serial.STOPBITS_ONE,
      bytesize=serial.EIGHTBITS
)
ser.dtr=True
ser.rts=True
ser.isOpen()
ser.reset_input_buffer()

print ("PDX port initialized Ok")
#*----------------------------------------*
#* Initialize WSJT-X end                  *
#*----------------------------------------*
ser1 = serial.Serial(
    port=wsjtPort,
    baudrate=wsjtBaud,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)
ser1.dtr=True
ser1.rts=True
ser1.isOpen()
ser1.reset_input_buffer()
print ("WSJT-X port initialized Ok")
print ("Ready to operate\r\n")

if log==True:
   with open('picoCAT.log', 'w') as f:
      now = datetime.now() # current date and time
      date_time = now.strftime("%m/%d/%Y, %H:%M:%S")
      f.write('picoCAT logging - session start at '+date_time+'\n')

input=1
out=''
#*-------------------------------------------*
#* Back to back loop, bridge and debug print *
#*-------------------------------------------*
while 1 :

#*--- First check WSJT-X

    if ser1.inWaiting() > 0:
       a = ser1.read(ser1.inWaiting())
       print('<WSJTX>=' + a.decode("utf-8"))
       ser.write(a)
       if log==True:
          with open('picoCAT.log', 'a') as f:
             now = datetime.now() # current date and time
             date_time = now.strftime("%m/%d/%Y, %H:%M:%S")
             f.write(str(date_time+' <WSJTX>=' + a.decode("utf-8"))+'\n')


#*--- Now check PDX, if it is a debug message do not resend it

    if ser.inWaiting() > 0:
       x = ser.read(ser.inWaiting())
       if x.decode("utf-8")[0:1]=="@" :
          print(x.decode("utf-8"))
          if log==True:
             with open('picoCAT.log', 'a') as f:
                now = datetime.now() # current date and time
                date_time = now.strftime("%m/%d/%Y, %H:%M:%S")
                f.write(date_time+' '+str(x.decode("utf-8"))+'\n')
       else :
          ser1.write(x)
          print("<PDX  >:"+x.decode("utf-8"))
          if log==True:
             with open('picoCAT.log', 'a') as f:
                now = datetime.now() # current date and time
                date_time = now.strftime("%m/%d/%Y, %H:%M:%S")
                f.write(date_time+' '+str("<PDX  >:"+x.decode("utf-8"))+'\n')
