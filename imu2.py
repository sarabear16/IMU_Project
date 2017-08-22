'''
There are 3 ways to show the projection of the block
1) Euler Angles
2) Quaternion Angles
3) Input from raw data


Throughout the code, I have made comments on which lines to comment/uncomment 
depening on which method you choose. 

'''
#Importing modules/libraries 
from OpenGL.GL import *
from OpenGL.GLU import *
import pygame
from pygame.locals import *
import serialcp2110
from time import sleep
import math
from ctypes import c_float, c_int32, cast, byref, POINTER

#Initializing Variabes 
yaw_mode = False
ax = ay = az = 0.0
t1=0
q0 = float(1.0)
q1 = float(0.0)
q2 = float(0.0)
q3 = float(0.0)
integralFBx = float(0.0)
integralFBy = float(0.0)
integralFBz = float(0.0)

#Initializing USB port connection
serial = serialcp2110.Serial()
serial.open()
serial.setPin(6,0)
serial.setPin(8,0)
serial.setPin(8,1)


#Read Funtion 
def myRead(n):
    global serial 
    myBuffer=''
    while True:
        segment= (serial.read(n)).decode('utf-8') #decodes from a bitstring into a string
        myBuffer+=segment
        if ('OK' in myBuffer or "ERR" in myBuffer ):  
            break   
    return myBuffer

def resize(width, height):
    if height == 0:
        height = 1
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, 1.0 * width / height, 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

def init():
    glShadeModel(GL_SMOOTH)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)


#Initialization of the IMU through AT Commands

def init_imu():
    global serial 
    serial.write("At+Ec=0\r\n")                     #Returns output line only
    print(myRead(2))

    serial.write("At+I2CC=0,1,1,3,4\r\n")           #Allocating SDA/SCL pins
    print(myRead(2))

    serial.write("At+I2CRW=0,0x28,0,3D00\r\n")      #Select BNO055 config mode
    print (myRead(2))

    serial.write("At+I2CRW=0,0x28,0,0701\r\n")      #Select Page1 to configure sensors
    print (myRead(2))

    serial.write("At+I2CRW=0,0x28,0,080D\r\n")      #Acc (cfg)
    print (myRead(2))

    serial.write("At+I2CRW=0,40,0,0A18\r\n")        #Gyro (cfg1)
    print (myRead(2))

    serial.write("At+I2CRW=0,0x28,0,0B00\r\n")      #Normal OpMode
    print (myRead(2))

    serial.write("At+I2CRW=0,0x28,0,0906\r\n")      #Mag (cfg)
    print (myRead(2))

    serial.write("At+I2CRW=0,0x28,0,0700\r\n")      #Page 0 to read sensors
    print (myRead(2))

    #serial.write("At+I2CRW=0,0x28,0,3B01\r\n")     #Select BNO055 sensor units- Euler(deg), Gyro (dps), Acc (mg)- Use this mode is evaluating Euler/Quaternion Angles
    #print (myRead(2))
    serial.write("At+I2CRW=0,0x28,0,3B03\r\n")      #Euler (deg), Gyro (rps), Acc (mg)
    print (myRead(2))

    serial.write("At+I2CRW=0,0x28,0,4000\r\n")      #Temp source
    print (myRead(2))

    serial.write("At+I2CRW=0,0x28,0,3E00\r\n")      #Pwr mode
    print (myRead(2))

    #serial.write("At+I2CRW=0,0x28,0,3D0C\r\n")     #Write Configuration to BNO55 Registers set to Fusion- Use this mode if evaluating Euler/Quaternion Angles 
    #print (myRead(2))
    serial.write("At+I2CRW=0,0x28,0,3D07\r\n")      #Op mode (AMG) <- non fusion
    print (myRead(2))

    serial.write("At+I2CRW=0,0x28,1,3D\r\n")        #Read back values to verify initializating is complete
    print (myRead(2))

#Funtion to display in GUI 
def drawtext(position, textstring):
    font = pygame.font.SysFont("Courier", 18, True)
    textsurface = font.render(textstring, True, (255, 255, 255, 255), (0, 0, 0, 255))
    textData = pygame.image.tostring(textsurface, "RGBA", True)
    glRasterPos3d(*position)
    glDrawPixels(textsurface.get_width(), textsurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)

#Function to display the block  
def draw():
    global ax,ay,az
    global rquad
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    glLoadIdentity()
    glTranslatef(0, 0.0, -7.0)

    osd_text = "pitch: " + str("{0:.2f}".format(ay)) + ", roll: " + str("{0:.2f}".format(ax))

    if yaw_mode:
        osd_line = osd_text + ", yaw: " + str("{0:.2f}".format(az))
    else:
        osd_line = osd_text

    drawtext((-2, -2, 2), osd_line)

    # the way I'm holding the IMU board, X and Y axis are switched,with respect to the OpenGL coordinate system
    
    if yaw_mode:  
        az=az+180  #Comment out if reading Euler Angle/Quaternion angles 
        glRotatef(az, 0.0, 1.0, 0.0)      # Yaw, rotate around y-axis

 
    glRotatef(ay, 1.0, 0.0, 0.0)          # Pitch, rotate around x-axis
    glRotatef(-1 * ax, 0.0, 0.0, 1.0)     # Roll, rotate around z-axis

    glBegin(GL_QUADS)
    glColor3f(1.0, 0.5, 0.0)
    glVertex3f(1.0, 0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(1.0, 0.2, 1.0)

    glColor3f(1.0, 0.5, 0.0)
    glVertex3f(1.0, -0.2, 1.0)
    glVertex3f(-1.0, -0.2, 1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(1.0, -0.2, -1.0)

    glColor3f(1.0, 0.0, 0.0)
    glVertex3f(1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, -0.2, 1.0)
    glVertex3f(1.0, -0.2, 1.0)

    glColor3f(1.0, 1.0, 0.0)
    glVertex3f(1.0, -0.2, -1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(1.0, 0.2, -1.0)

    glColor3f(0.0, 0.0, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(-1.0, -0.2, 1.0)

    glColor3f(1.0, 0.0, 1.0)
    glVertex3f(1.0, 0.2, -1.0)
    glVertex3f(1.0, 0.2, 1.0)
    glVertex3f(1.0, -0.2, 1.0)
    glVertex3f(1.0, -0.2, -1.0)
    glEnd()

#This function reads the Euler angle readings from BNO055
def BNO55_ReadEuler(): 
    global ax, ay, az
    serial.write("At+I2CRW=0,0x28,6,1A\r\n") #starts at 1A and works up the register map 
    x= myRead(2)

    if count<0:
        x=x.split(',')[1]
        y=x.split(' ') [0:6]
        y = [int(a,16) for a in y]      

        zz=((y[1]<<8)+y[0])
        if zz>0x7FFF:
            zz=(zz-0x10000)
        az=-(zz/16.0)

        yy=((y[3]<<8)+y[2])
        if yy>0x7FFF:
            yy=(yy- 0x10000)
        ax=-(yy)/16.0 #changed the ax/ay becasue on the program it was flipped, add the negative bc the code was backwards

        xx=((y[5]<<8)+y[4])
        if xx>0x7FFF:
            xx=(xx - 0x10000)
        ay=xx/16.0


#This function reads the Quaternion angle readings from the BnO055
def BNO55_readQuaterion():
    global ax, ay, az
    serial.write("At+I2CRW=0,0x28,8,20\r\n") #Reads 8 bytes starting at register 20 for quaterion angles 
    s=myRead(2) #calls MyRead function to return after it is written
    if count<0:
        s=s.split(',')[1]
        s=s.split(' ') [0:8]
        s = [int(a,16) for a in s] 

        ww=((s[1]<<8)+s[0])
        if ww>0x7FFF:
            ww=(ww-0x10000)
        w=(ww)/float(1<<14)

        xx=((s[3]<<8)+s[2])
        if xx>0x7FFF:
            xx=(xx-0x10000)
        y=(xx)/float(1<<14)
            
        yy=((s[5]<<8)+s[4])
        if yy>0x7FFF:
            yy=(yy-0x10000)
        x=(yy)/float(1<<14)

        zz=((s[7]<<8)+s[6])
        if zz>0x7FFF:
            zz=(zz-0x10000)
        z=(zz)/float(1<<14) 

        #Turns the Quaternion readings into Euler Angles for projection
        ysqr = y*y
        t0 = +2.0 * (w * x + y*z)
        t1 = +1.0 - 2.0 * (x*x + ysqr)
        ax = (math.degrees(math.atan2(t0, t1)))
        
        t2 = +2.0 * (w*y - z*x)
        t2 =  1 if t2 > 1 else t2
        t2 = -1 if t2 < -1 else t2
        ay= math.degrees(math.asin(t2))
       
        t3 = +2.0 * (w * z + x*y)
        t4 = +1.0 - 2.0 * (ysqr + z*z)
        az=math.degrees(math.atan2(t3, t4))

#This function takes raw data from the BNO055_readGyroMagAccel() function into an algorithm that produces the Euler Angle readings for projection
def Bno055_convert(values):
    if values is not None: 
        global q0, q1, q2, q3, integralFBx, integralFBz, integralFBy,ax,ay,az
        twoKp = (2.0 * 0.5) // 2
        twoKi = (2.0 * 0.1) // 2
        halfex = float(0.0)
        halfey = float(0.0)
        halfez = float(0.0)
        time_diff=values[0]
        ax=values[1]
        ay=values[2]
        az=values[3]
        mx=values[4]
        my=values[5]
        mz=values[6]
        gx=values[7]
        gy=values[8]
        gz=values[9]
        

      #Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0
        q0q1 = q0 * q1
        q0q2 = q0 * q2
        q0q3 = q0 * q3
        q1q1 = q1 * q1
        q1q2 = q1 * q2
        q1q3 = q1 * q3
        q2q2 = q2 * q2
        q2q3 = q2 * q3
        q3q3 = q3 * q3
      
      # Use magnetometer measurement only when valid (avoids NaN in magnetometer normalisation)
        if((mx != 0.0) and (my != 0.0) and (mz != 0.0)):
            #float hx, hy, bx, bz
            #float halfwx, halfwy, halfwz

            #Normalise magnetometer measurement
            recipNorm = invSqrt(mx * mx+ my * my + mz * mz)
            mx *= recipNorm
            my *= recipNorm
            mz *= recipNorm
            
            #Reference direction of Earth's magnetic field
            hx = 2.0 * (mx * (0.5 - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2))
            hy = 2.0 * (mx * (q1q2 + q0q3) + my * (0.5- q1q1 - q3q3) + mz * (q2q3 - q0q1))
            bx = math.sqrt(hx * hx + hy * hy)
            bz = 2.0 * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5 - q1q1 - q2q2))
            
            #Estimated direction of magnetic field
            halfwx = bx * (0.5 - q2q2 - q3q3) + bz * (q1q3 - q0q2)
            halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3)
            halfwz = bx * (q0q2 + q1q3) + bz * (0.5 - q1q1 - q2q2)
            
            # Error is sum of cross product between estimated direction and measured direction of field vectors
            halfex = (my * halfwz - mz * halfwy)
            halfey = (mz * halfwx - mx * halfwz)
            halfez = (mx * halfwy - my * halfwx)

      #Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if((ax != 0.0) and (ay != 0.0) and (az != 0.0)): 
      
            #Normalise accelerometer measurement
            recipNorm = invSqrt(ax * ax + ay * ay + az * az)
            ax *= recipNorm
            ay *= recipNorm
            az *= recipNorm
            
            #Estimated direction of gravity
            halfvx = q1q3 - q0q2
            halfvy = q0q1 + q2q3
            halfvz = q0q0 - 0.5 + q3q3
          
            #Error is sum of cross product between estimated direction and measured direction of field vectors
            halfex += (ay * halfvz - az * halfvy)
            halfey += (az * halfvx - ax * halfvz)
            halfez += (ax * halfvy - ay * halfvx)
      
      # Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
        if (halfex != 0.0 and halfey != 0.0 and halfez != 0.0):

        # Compute and apply integral feedback if enabled
            if(twoKi > 0.0):
                integralFBx += twoKi * halfex * (time_diff) #integral error scaled by Ki
                integralFBy += twoKi * halfey * ( time_diff)
                integralFBz += twoKi * halfez * ( time_diff)
                gx += integralFBx #apply integral feedback
                gy += integralFBy
                gz += integralFBz
            
            else:
                integralFBx = 0.0  #prevent integral windup
                integralFBy = 0.0
                integralFBz = 0.0
            
            #Apply proportional feedback
            gx += twoKp * halfex
            gy += twoKp * halfey
            gz += twoKp * halfez
      
        #Integrate rate of change of quaternion
        gx *= (0.5 * (time_diff))  # pre-multiply common factors
        gy *= (0.5 * (time_diff))
        gz *= (0.5 * (time_diff))
        qa = q0
        qb = q1
        qc = q2
        q0 += (-qb * gx - qc * gy - q3 * gz)
        q1 += (qa * gx + qc * gz - q3 * gy)
        q2 += (qa * gy - qb * gz + q3 * gx)
        q3 += (qa * gz + qb * gy - qc * gx)
          
        #Normalise quaternion
        recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        q0 *= recipNorm
        q1 *= recipNorm
        q2 *= recipNorm
        q3 *= recipNorm
        #print (q0, q1,q2,q3)

        x=q0
        y=q1
        z=q2
        w=q3

        #Convert the raw reading values into Euler Angle Readings 

        ysqr = y*y
        t0 = +2.0 * (w * x + y*z)
        t1 = +1.0 - 2.0 * (x*x + ysqr)
        az = -(math.degrees(math.atan2(t0, t1))) #negative bc the directions are changed

        
        t2 = +2.0 * (w*y - z*x)
        t2 =  1 if t2 > 1 else t2
        t2 = -1 if t2 < -1 else t2
        ax= -(math.degrees(math.asin(t2))) #swapped values around bc not oriented correctly 
       
        t3 = +2.0 * (w * z + x*y)
        t4 = +1.0 - 2.0 * (ysqr + z*z)
        ay=(math.degrees(math.atan2(t3, t4)))
        #print(ax,ay,az)


#This function takes in the readings frm the Gyro/Acc/Mag
def BNO055_readGyroMagAccel():
    global count
    global t1
    global countx
    serial.write("At+I2CRW=0,0x28,18,08\r\n")
    s=myRead(2)
    #print(s)

    if count<0:
        sa=s.split(',') 
        new=sa[1]
        new=new.split('\r\n')
        new=new[0:2]
        new=''.join(new)
        new=new.split(' ') [0:18]
        new=[int(a,16) for a in new] 
        #print(new)
        
        time=sa[0] #time split to get the time stamp 
        time=time.split(':')[1]
        #print(time)
        t2=int(time,16)
        #print(t2)
        time_diff=(t2-t1)/100000.0
        #print (1/time_diff)
        t1=t2 
        #count1+=1
        #print(count1)


        if (count>-4):  #Count used for callibration of the device 
            print('')
            
        else:
        
            accx=((new[1]<<8)+new[0])
            if accx>0x7FFF:
                accx=(accx-0x10000)
            accx=(accx)/float(1.0)
           
            accy=((new[3]<<8)+new[2])
            if accy>0x7FFF:
                accy=(accy-0x10000)
            accy=(accy)/float(1.0)
                
            accz=((new[5]<<8)+new[4])
            if accz>0x7FFF:
                accz=(accz-0x10000)
            accz=(accz)/float(1.0)

            magx=((new[7]<<8)+new[6])
            if magx>0x7FFF:
                magx=(magx-0x10000)
            magx=(magx)/float(16.0) 

            magy=((new[9]<<8)+new[8])
            if magy>0x7FFF:
                magy=(magy-0x10000)
            magy=(magy)/float(16.0) 

            magz=((new[11]<<8)+new[10])
            if magz>0x7FFF:
                magz=(magz-0x10000)
            magz=(magz)/float(16.0) 

            gyrox=((new[13]<<8)+new[12])
            if gyrox>0x7FFF:
                gyrox=(gyrox-0x10000)
            gyrox=(gyrox)/float(900.0)


            gyroy=((new[15]<<8)+new[14])
            if gyroy>0x7FFF:
                gyroy=(gyroy-0x10000)
            gyroy=(gyroy)/float(900.0)


            gyroz=((new[17]<<8)+new[16])
            if gyroz>0x7FFF:
                gyroz=(gyroz-0x10000)
            gyroz=(gyroz)/float(900.0) 

         
            return[time_diff, accx, accy, accz, magx, magy, magz, gyrox, gyroy, gyroz]

#Calculates the Inverse square root of a number used in the Bno055 convert function
def invSqrt(number):
    threehalfs = 1.5
    x2 = number * 0.5
    r = c_float(number)
    i = cast(byref(r), POINTER(c_int32)).contents.value
    i = c_int32(0x5f3759df - (i >> 1))
    r = cast(byref(i), POINTER(c_float)).contents.value
    r = r * (1.5 - (x2 * r* r))
    return r
       
count=1
def main():
    global ax,ay,az
    global yaw_mode
    global serial 
    global count
    video_flags = OPENGL | DOUBLEBUF
    pygame.init()
    screen = pygame.display.set_mode((640, 480), video_flags)
    pygame.display.set_caption("Press Esc to quit, z toggles yaw mode")
    resize(640, 480)
    init()
    init_imu()
    frames = 0
    ticks = pygame.time.get_ticks()
    
    while 1:
        event = pygame.event.poll()
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            break
        if event.type == KEYDOWN and event.key == K_z:
            yaw_mode = not yaw_mode
        
        pygame.display.flip()
        frames = frames + 1

        #BNO55_ReadEuler()                #If you want to read Euler Angles, comment out BNO55_readQuaterion(), data=BNO055_readGyroMagAccel() and, Bno055_convert(data) 
        #BNO55_readQuaterion()            #If you want to read quaternion angles, comment out data=BNO055_readGyroMagAccel() and, Bno055_convert(data), and read Euler
        data=BNO055_readGyroMagAccel()    #If you want to read raw values, comment out the two functions above
        Bno055_convert(data) 
        count-=1
        draw()
        #zeroGyro()
           

if __name__ == '__main__': main()
