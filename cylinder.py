#import picamera
from PIL import Image

import numpy as np
#import Rpi.GPIO as IO
imagecount = 0
#pc = picamera.PICamera()
#pc.resolution = ( 64 , 64 )
inputPin,rPin,gPin,bPin = 1,2,3,4
#def setup():
#    [IO.setup(pin,IO.OUT) for pin in [rPin,gPin,bPin]]
#    IO.setup( inputPin , IO.IN )



def getCylinderColor():  
    R , G , B = 0 , 0 , 0
#    img_name = "cylinder.jpg"
#    pc.capture( img_name ) 
    with Image.open( "b.jpg" ) as img:
        img = img.crop(( 160 , 160 , 480 , 480 ))       # crop the image 'n form a 32 x 32 image
        a = np.asarray( img )
        for  i in range( len( a ) ):
            for  j in range( len( a ) ):
                R += a[ i ][ j ][ 0 ]
                G += a[ i ][ j ][ 1 ]
                B += a[ i ][ j ][ 2 ]
        dominant = max( [ R , G , B ] )
        print(R)
        print(G)
        print(B)
        
        if( dominant == R ):
            return 0
        elif( dominant == G ):
            return 1
        else:
            return 2
#setup()

#while(True):
#    if(IO.input(inputPin)==True):
#        break
    
out = getCylinderColor()
"""
if(out==0):
    IO.output(rPin,True)
    IO.output(gPin,False)
    IO.output(bPin,False)
elif(out==2):
    IO.output(rPin,False)
    IO.output(gPin,True)
    IO.output(bPin,False)
else:
    IO.output(rPin,False)
    IO.output(gPin,False)
    IO.output(bPin,True)
"""

