import time
import os
import UR5

photoFunc = {1:"Photoneo"}
    
if __name__=="__main__":
    print("Pls make sure your device has already connected")
    roboType = input('Pls Enter the number of robot type: 1.UR5\n')
    while(True):	
	if(roboType!=1):
	    roboType = input('Pls Enter the number of robot type: 1.UR5\n')
	else: 
	    robofunc=getattr(UR5,'UR5')
	    break
    photoType = input('Pls enter the number of photograph type: 1.Photoneo\n')
    while(True):	
	if(photoType!=1):
	    photoType = input('Pls enter the number of photograph type: 1.Photoneo\n')
	else: 
	    break
    num = input('Pls enter the total number of tests\n')
    while(True):	
	if(num<=0):
	    num = input('Pls enter the total number of tests\n')
	else: 
	    break
    
    photofunc=photoFunc[photoType]
    timestamp=time.strftime('data/%Y-%m-%d/%H:%M:%S',time.localtime(int(round(time.time()*1000))/1000))
    folder=time.strftime('data/%Y-%m-%d',time.localtime(int(round(time.time()*1000))/1000))
    if not os.path.exists(folder):
	os.makedirs(folder)
    cnt=1    
    
    while num:
        os.system(r'./'+photofunc+r' '+timestamp+r' '+r'_start'+str(cnt))
        robofunc(timestamp,"_start"+str(cnt),True)
        os.system(r'./'+photofunc+r' '+timestamp+r' '+r'_end'+str(cnt))
	robofunc(timestamp,"_end"+str(cnt),False)
        num=num-1
	cnt=cnt+1
        time.sleep(1)
        



