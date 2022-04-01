import serial
import time
import numpy as np
from scipy.fftpack import fft
import matplotlib.pyplot as plt
import serial.tools.list_ports as p

all_comports = serial.tools.list_ports.comports()

for comport in all_comports:
    print(comport.device, comport.name, comport.description, comport.interface)
print("Ender your port name, ie COM37")
pname=input()
print("selected port",pname)
print("Enter yor UART speed,default=115200")
pspeed=input();
if pspeed == "":
   pspeed=115200
print("the speed of ",pname, "is equal to", pspeed)
print()
print("Please enter the sampling window size, i.e. 2048")
pN=input();
N=int(pN)
print("N=",N)

print("Please enter tSCALE emty is for adaptive,  A for 30 mV, B for 20, C for 10 mV D for 5 mV")
pscale=input()
if pscale == "":
    scale=1.0
if pscale == "A":
    scale=0.03
if pscale == "B":
    scale=0.02
if pscale == "C":
    scale=0.01
if pscale == "D":
    scale=0.003
if scale < 0.5:
    print("scale=",scale)
if scale >  0.5:
    print("adaptive amplitude")

    


    


plt.style.use('seaborn-poster')
from drawnow import *

k=0
j=0
T =0.004

Np1=N+1
Nm1=N-1
mode = 0

Xf = np.linspace(0.0,1.0/(2.0*T), N)
Xt = np.linspace(0.0, N*T, N)
Yf = np.linspace(0.0, N*T, N)
ch0 = []
data_ch0=0.0

plt.ion()
def makeFig():
   plt.figure(1)
   plt.tight_layout()
   plt.subplot(211)
   plt.xlabel('time in Seconds ',fontsize=10)
   plt.title('Time domain 250 Hz sampling',fontsize=14)
   if scale < 0.5:
      # plt.yticks(np.arange(-scale,scale,step=0.002))
       plt.ylim(-scale,scale)
   plt.grid(True)
   plt.ylabel('iþaret gücü Volt',fontsize=10)
   plt.plot(Xt,ch0, 'r-',label='time',lw=0.8)
   plt.legend(loc='upper right')

   plt.subplot(212)
   plt.xlabel('frequency in Hz',fontsize=10)
   #plt.xticks(np.arange(0,N/(2.0*T),step=1))
   plt.title('')
   plt.grid(True)
   plt.ylabel('FFT Volt', fontsize=10)
   plt.plot(Xf[0:100],abs(Yf[0:100]), 'g-',label='Frequency',lw=0.8)
   plt.legend(loc='upper right')

raw=serial.Serial(pname,pspeed)
raw.reset_input_buffer()
time.sleep(0.1)
print("in waiting:",raw.in_waiting)

try:
        read_sync = True
        i = 0
        while True:

                if read_sync:
                        sy = raw.read()
                        if sy[0] == 0xA:
                                read_sync = False
                        else:
                                continue

                inw = raw.in_waiting
                #print(inw)
                if inw < 3:
                        continue

                read_n = int(inw / 3) * 3
                data = raw.read(read_n)
                while(k<read_n):
                  #  print(k,data[k]+data[k+1]*256,data[k+2]+data[k+3]*256,data[k+4]+data[k+5]*256)
                    data_ch0=data[k]+data[k+1]*256
                    if(data_ch0 >32767):
                        data_ch0=data_ch0-65536
                    data_ch0=data_ch0*0.000001007080078125 ##result is volt
                    ch0.append(data_ch0)
                    k=k+3
                    j=j+1
                    if(j > N):
                        mode = 1
                        ch0.pop(0)
                        j=Np1
                i=i+1
                if i == 20:
                    if mode == 1:
                        Yf=fft(ch0)
                        drawnow(makeFig)

                    i=0
                k=0
                read_sync = True


except KeyboardInterrupt:
        print("stopping...")
        raw.close()

print("Done")
