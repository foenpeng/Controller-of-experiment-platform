# This version came after v9.data_acquisition and will be directly updated on github. 2/8/2017

import cv2
import winsound
import numpy as np
# np.set_printoptions(threshold=np.nan)
import datetime
import os
import sys
import serial as s
import threading
import time as t
from os import listdir, mkdir, remove
from shutil import move
from datetime import date
from threading import Thread

class videoDetection(Thread):

    def __init__(self):
        Thread.__init__(self)
        self.moth_prnt = False
        self.running = True
        self.start_time = None
        self.moth_move = False   
        self.ROI = [300,300,150] # x,y,r
        self.fps = 5
        self.wait_time = 200; # 1000/fps
        self.min_area = 500;
        self.image_threshold = 50
        self.Mfilename = "m_data.csv"
        self.rawvideo = "video.avi"
        self.Mfilename = f.folder + "/" + self.Mfilename
        self.rawvideo = f.folder + "/" + self.rawvideo
        self.cap = cv2.VideoCapture(0)
        self.fourcc = cv2.cv.CV_FOURCC('X','V','I','D')
        self.video  = cv2.VideoWriter(self.rawvideo,self.fourcc, self.fps, (640, 480), False)
        self.firstFrame = None  
        self.MothLeft = True  # flag to judge whether moth really left
        self.AbsentFrame = 0    
        self.InjectionDelay = 2 # how many seconds delay to refill after moth left        

        
    def begin(self):
        t.sleep(1.5)
        while(True):
            ret,frame1 = self.cap.read()
            self.gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
            if self.firstFrame is None:
                self.firstFrame = self.gray1
                continue
            subt1 = cv2.absdiff(self.firstFrame,self.gray1)
            self.firstFrame = self.gray1
            position =  np.matrix(np.where(subt1>30))
            if position.shape[1]<50:
                break            
         
    def run(self):    
        results = []
        Mfile=open(self.Mfilename,'w')
        self.begin()
        self.video.write(self.gray1)

        
        while(True):
            # Capture frame-by-frame
            ret, frame = self.cap.read()
            toc = round(t.clock(),2)
            if self.start_time is None:
                self.start_time = toc
            # Our operations on the frame come here
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            subt = cv2.absdiff(gray,self.gray1)
            ret,thresh = cv2.threshold(subt,self.image_threshold,255,cv2.THRESH_BINARY)
            thresh_dilate = cv2.dilate(thresh, None, iterations=2)
            (cnts, _) = cv2.findContours(thresh_dilate, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            
             # loop over the contours to find the biggest moving object
            biggest_cnt = 0
            num_biggest_cnt = 0

            for c in cnts:
                 # if the contour is too small, ignore it
                if cv2.contourArea(c) < self.min_area:
                    continue
                if cv2.contourArea(c) > biggest_cnt:
                    biggest_cnt = cv2.contourArea(c)
                    num_biggest_cnt = c
                             
             #find the centroid of the moving object
            centroid_x = 0
            centroid_y = 0

            if hasattr(num_biggest_cnt,"__len__"):   # judge if num_biggest_cnt is still 0
                M = cv2.moments(num_biggest_cnt)
                centroid_x = int(M['m10']/M['m00'])
                centroid_y = int(M['m01']/M['m00'])
                cv2.circle(gray, (centroid_x,centroid_y), 5, (0,0,0), -1)
                if self.moth_move  ==  False:
                     print("Moth is moving")
                     self.moth_move = True
            else:
                self.moth_move = False
             
            # Determine whether moth is present or not and draw circles
            if (self.ROI[0]-centroid_x)**2 + (self.ROI[1]-centroid_y)**2 < self.ROI[2]**2: # judge if the centroid is inside the circle
                cv2.circle(gray, (self.ROI[0], self.ROI[1]), self.ROI[2], (255, 255, 255), 2)
                cv2.putText(gray, "Moth Present", (250, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
                self.moth_prnt = True
                self.AbsentFrame = 0
                val = 1
                line = "{0},{1}\n".format(val,toc)
                Mfile.write(line)                      
            else:
                cv2.circle(gray, (self.ROI[0], self.ROI[1]), self.ROI[2], (0, 0, 0), 2)
                self.moth_prnt = False
                self.AbsentFrame += 1
                val = 0
                line = "{0},{1}\n".format(val,toc)
                Mfile.write(line)
            
            # Determine whether moth really left or not, in order to injection
            if self.AbsentFrame > self.fps * self.InjectionDelay:
                self.MothLeft = True
            else:
                self.MothLeft = False
            
            cv2.putText(gray, datetime.datetime.now().strftime("%A %d %B %Y %I:%M:%S%p"),(10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
            cv2.putText(gray, "Time Elapsed: " + str(toc),(250, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
            if f.nct_prnt:
               cv2.putText(gray, "Nectar Present ",(10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
               
            cv2.putText(gray, "No. Injections : {0}".format(num_inject) ,(400, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
            cv2.imshow('frame',gray)
            cv2.imshow("thresh",thresh)
            self.video.write(gray)
            if cv2.waitKey(self.wait_time) & self.running == False:
                break
                
        Mfile.close()
        self.cap.release()
        self.video.release()
        cv2.destroyAllWindows()
                
class flowerController(Thread):

    def __init__(self, port, accel_sample_freq):
        Thread.__init__(self)
        # Declare filenames to write in output folder
        self.Xfilename = "x_data.csv"
        self.Yfilename = "y_data.csv"
        self.Zfilename = "z_data.csv"
        self.Nfilename = "n_data.csv"
        self.Efilename = "e_data.csv"
        self.Ifilename = "i_data.csv"
        self.rawfilename = "raw_data"
        self.path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.nct_prnt = False
        self.start_time = None
        self.e_time = 0      
        # Get the morphology to name the output folder
        self.morph = str(raw_input("Which morphology is it?\n")) + "l070r1.5R025v020p000"
        os.chdir(os.path.join(self.path, "Data Files"))
        if not os.path.exists((os.path.join(os.getcwd(), self.morph))):
           os.makedirs((os.path.join(os.getcwd(), self.morph)))
        os.chdir(os.path.join(os.getcwd(), self.morph))
        today = date.today()
        now = datetime.datetime.now()
        # Get the date, also used to name output folder
        self.folder = str(today)+ "_" + str(now.hour)+ "_" + str(now.minute) +"_" + self.morph
        # change working directory to data files
        
        try:
            mkdir(self.folder)
        except:
            print("failed to make working directory\n")
        # Open output files in working directory
        self.Xfilename = self.folder + "/" + self.Xfilename
        self.Yfilename = self.folder + "/" + self.Yfilename
        self.Zfilename = self.folder + "/" + self.Zfilename
        self.Nfilename = self.folder + "/" + self.Nfilename
        self.Efilename = self.folder + "/" + self.Efilename
        self.Ifilename = self.folder + "/" + self.Ifilename
        self.rawfilename = self.folder + "/" + self.rawfilename

        self.port = port
        self.running = False

        self.accel_sample_freq = accel_sample_freq
    def begin(self):
        try:
            # Open port at 1Mbit/sec
            self.ser = s.Serial(self.port,
                                    1000000,
                                    timeout = 1)
            success = True
            # Assert Data Terminal Ready signal to reset Arduino
            self.ser.rtscts = True
            self.ser.dtr = True
            t.sleep(1)
            self.ser.dtr = False
            t.sleep(2)
        except s.SerialException:
            success = False
            print("Failed to open " + self.port + "\n")
            raise(s.SerialException)

        if success:
            try:
                # Open output files for wriging
                self.Xfile = open(self.Xfilename, 'w')
                self.Yfile = open(self.Yfilename, 'w')
                self.Zfile = open(self.Zfilename, 'w')
                self.Nfile = open(self.Nfilename, 'w')
                self.Efile = open(self.Efilename, 'w')
                self.Ifile = open(self.Ifilename, 'w')
                self.raw_data = open(self.rawfilename, 'wb')
                # Send samples rates and start command
                cmd = bytearray("{0}\n".format(self.accel_sample_freq), 'ascii')
                self.ser.write(cmd)
                success = True
            except BaseException:
                success = False
                raise(e)
        if success:
            self.running = True
        else:
            self.running = False

    """
    This function implements the running Loop of the
    FlowerController thread. It waits for 3-byte frames
    of data from the arduino and writes them to a separate
    file based on the first byte, the code, of the data.
    """
    def run(self):
        self.begin()
        self.ser.flushInput()
        high_to_low = 100
        low_to_high = 150
        self.nct_prnt = False
        if self.start_time is None:
                self.start_time = round(t.clock(),5)
        while self.running:
            # Read 3-bytes from the serial port
            while(self.ser.in_waiting):
                # Read two frames worth of data and write to raw file
                data = self.ser.read(24)
                self.raw_data.write(data)

                # Parse the data for a nectar measurement
                nectar_value = None
                for i in range(len(data)):
                    # Check for the data code
                    if data[i] == 'N' and i+1 in range(len(data)):
                        if i-3 in range(len(data)):
                            # Check to make sure that the previous value was from Z channel
                            if data[i-3] == 'Z':
                                nectar_value = data[i+1]
                                break
                        if i+3 in range(len(data)):
                            # Check to make sure that the following value is form X channel
                            if data[i+3] == 'X':
                                nectar_value = data[i+1]
                                break
                # Determine the nectar state
                if nectar_value is not None:
                        toc = round(t.clock(),3)
                        if toc - self.e_time > 1:
                            if(self.nct_prnt == True and (ord(nectar_value) > low_to_high)) :
                                self.nct_prnt = False
                                print("Nectar emptied at time stamp {0} \n".format(toc))
                                line = "0,{0}\n".format(toc)
                                self.Efile.write(line)
                                self.e_time = toc
                            elif(self.nct_prnt == False and (ord(nectar_value) < high_to_low)):
                                self.nct_prnt = True
                                print("Nectar filled at time stamp {0}\n".format(toc))
                                line = "1,{0}\n".format(toc)
                                self.Efile.write(line)    
                                self.e_time = toc                            


        self.stop()

    def stop(self):
        # Assert Data Terminal Ready to reset Arduino
        self.ser.dtr = True
        t.sleep(1)
        self.ser.dtr = False
        # Close the port
        self.ser.close()
        # Unpack the data
        self.unpack_data()
        # Fix the time overflow issue
        try:

            update_time(self.Xfilename, 4 * self.accel_sample_freq)
            update_time(self.Yfilename, 4 * self.accel_sample_freq)
            update_time(self.Zfilename, 4 * self.accel_sample_freq)
            update_time(self.Nfilename, 4 * self.accel_sample_freq)

            pass
        except OSError as e:
            raise(e)
        # Write the comments file
        try:
            commentfile = self.folder + "/comments.txt"

            filename = open(commentfile, 'w')
            filename.write("Flower morphology tested: \n")
            filename.write("{0}\n".format(self.morph)) # Flower morphology tested: 
            filename.write("Trial starts at: \n")
            filename.write("{0}\n".format(starttime)) # Start time
            #filename.write("Video starts at time: {0}\n".format(v.start_time))
            #filename.write("Sensor starts at time: {0}\n".format(self.start_time))
            #toc = round(t.clock(),3)
            filename.write("Program lasts (seconds): \n")
            filename.write("{0}\n".format(exit_time)) # Program lasts time
            filename.write("The x,y,z sampling frequency: \n")
            filename.write("{0}\n". format(self.accel_sample_freq)) # The x,y,z sampling frequency
            temp = raw_input("Temperature? \n").strip()
            filename.write("Temperature? \n")
            filename.write("{0}\n".format(temp))
            hum = raw_input("Humidity? \n").strip()
            filename.write("Humidity? \n")
            filename.write("{0}\n".format(hum))
            filename.write("Sex of the moth? \n\n")
            weight = raw_input("Body Weight? \n").strip()
            filename.write("Body Weight?\n")
            filename.write("{0}\n".format(weight))
            filename.write("Body length? \n\n")
            filename.write("Proboscis length? \n\n")
            filename.write("How many days after eclosion? \n\n")
            filename.write("Program exit condition?\n")
            filename.write(exit_text +"\n")
            comments = raw_input("Comments on this trial?\n").strip()
            filename.write(comments)
            filename.close()
        except OSError as e:
            raise(e)

    """
    This function reads from the raw binary data file and separates each
    data frame, where a frame consists of one read from each analog input channel
    and its corresponding timestamp.

    After collecting the frames, it will compute the amount of bytes lost in
    transmission, and then write the data to appropriate files (one for each channel)
    """
    def unpack_data(self):
        # Close the raw_data file, since it was open for writing previously
        self.raw_data.close()
        # Open the raw data file for reading
        self.raw_data = open(self.rawfilename, 'rb')
        data = bytearray()

        # Iterate over the file, reading each byte and appending to an array
        while True:
            byte = self.raw_data.read(1)
            if byte:
                data += byte
            else:
                break

        index = 0                        # iteration index
        frames = []                        # Location of frames in the data array

        while(index + 11 < len(data)):    # Verify that there is a full frame yet to be processed
            if locate_frame(index,data):
                frames.append(index)    # Append this index to a list of indexes pointing to valid frames
                index += 12                # Increment by the size of one frame in bytes
            else:
                index += 1                # Increment by one byte
        
        self.frame_count = len(frames)
        print("frame count", self.frame_count)
        print("bytes received", len(data))
        bytes_lost = len(data) - len(frames) * 12
        print("bytes lost", bytes_lost)
        print("loss ratio " + str(round(100 * bytes_lost / float(len(data)),4)) + " percent" )


        for frame in frames:
            # Write the X value and timestamp to the CSV file
            value = data[frame+1]
            timestamp = data[frame+2]
            line = "{0},{1}\n".format(value,timestamp)
            self.Xfile.write(line)

            # Write the Y value and timestamp to the CSV file
            value = data[frame+4]
            timestamp = data[frame+5]
            line = "{0},{1}\n".format(value,timestamp)
            self.Yfile.write(line)

            # Write the Z value and timestamp to the CSV file
            value = data[frame+7]
            timestamp = data[frame+8]
            line = "{0},{1}\n".format(value,timestamp)
            self.Zfile.write(line)

            # Write the N value and timestamp to the CSV file
            value = data[frame + 10]
            timestamp = data[frame + 11]
            line = "{0},{1}\n".format(value,timestamp)
            self.Nfile.write(line)

        # Close the output files
        self.Xfile.close()
        self.Yfile.close()
        self.Zfile.close()
        self.Nfile.close()
        self.Efile.close()
        self.Ifile.close()

def locate_frame(index,data): 
    if data[index] != ord('X'):
        return False
    elif data[index + 3] != ord('Y'):
        return False
    elif data[index + 6] != ord('Z'):
        return False
    elif data[index + 9] != ord('N'):
        return False
    else:
        return True


"""
This function is used to adjust the time stamps sent by
the flower controller after the data has been unpacked and sorted
into separate files.

For example, the sequence of time stamps
0, 1, ..., 2^16-1, 0, 1, ... 2^16-1 is converted into
0, 1, ..., 2^16-1, 2^16, 2^16+1, ..., 2^17-1
"""
def update_time(filename, frequency):
    print("Updating " + filename)
    # open the file for reading
    try:
        in_file = open(filename, 'r')
    except OSError:
        raise OSError("Error opening " + filename + "for reading!")
    # Loop over the file contents, and then process
    lines = list()
    offset = 0;
    last_time = 0;
    modulus = pow(2,8)
    time_unit = (exit_time - f.start_time)/f.frame_count
    time_line = f.start_time
    for line in in_file:
        # split the data into (data,time_stamp)
        (data, sep, time_stamp) = line.partition(',')
        time_line = time_line + time_unit
        time_stamp = str(round(time_line,4))
        # Rejoin the parts to a new line, and write to file
        if('' == data):
            lines.append(time_stamp+'\n')
        else:
            lines.append("".join((data,sep,time_stamp))+'\n')
    in_file.close()
    # Open the file for writing
    try:
        out_file = open(filename,'w')
    # OSError caught incase the file does not exist or this
    # script does not have permissions to write
    except OSError:
        print("Error opening " + filename + " for writing!")
        raise OSError
    # Write the data to the file
    for line in lines:
        out_file.write(line)
    # All done with this file
    out_file.close()
    
def injection(nct_vlm):
    Nec_ser.flushInput()                                                     
    cmd = bytearray("VOLUME:{0}\n".format(nct_vlm), 'ascii')
    Nec_ser.write(cmd)
    toc = round(t.clock(),3)
    time = str(toc)
    line = "{0}\n".format(time);
    print("Injection requested at time stamp {0}".format(time))
    Ifile.write(line)
    return toc
    
if __name__ == "__main__":
    accel_sample_freq = 1000
    #actual_accel_freq = int(16000000/(16000000/accel_sample_freq - 1))
    #print("Sample rate, {0}".format(actual_accel_freq))
    nct_vlm = 20
    max_waittime =360
    
    # default nectar port COM4 and data_acquisition port COM3
    try:
        # Open port at 1Mbit/sec
        Nec_ser = s.Serial('COM4',115200, timeout = 1)
        success = True
        # Assert Data Terminal Ready signal to reset Arduino
        Nec_ser.rtscts = True                                                       
        Nec_ser.dtr = True
        t.sleep(1)
        Nec_ser.dtr = False

    except s.SerialException:
        success = False
        print("Failed to open COM4\n")
        raise(s.SerialException)

    f = flowerController('COM3', accel_sample_freq)
    v = videoDetection()
    raw_input("enter anything to begin\n")
    print("output is being written to " + f.folder)

    f.start()
    v.start()
    starttime = datetime.datetime.now().strftime("%A %d %B %Y %I:%M:%S%p")
    
    Ifile = open(f.Ifilename,'w')
    
    injector_ready = False
    injector_response = ""
    num_inject = 0
    i_time = [0]
    while True:
        try:
            if t.clock() - f.e_time > max_waittime:
                exit_text = "Max waiting time reached"
                print(exit_text)
                break      
            #elif num_inject == 500/nct_vlm:
                #exit_time = round(t.clock(),3)
                #exit_text = "Max injection reached"
                #print(exit_text)
                #break 
            elif (0 == injector_response.find("HALT!")):
                #exit_time = round(t.clock(),3)
                exit_text = "Max injection reached"
                print("Max injection reached")
                break 
            elif len(i_time) > 4:
                if i_time[-1] - i_time[-4] < 10:
                    exit_text = "Repeated injection"
                    print(exit_text)
                    break    
                    
            if(Nec_ser.in_waiting>0):
                injector_response = Nec_ser.readline().rstrip('\n')
                print(injector_response)
                
                if(0 == injector_response.find("Waiting for user input...")):
                    injector_ready = True
                    t.sleep(2)
                    
            if  v.MothLeft == True and f.nct_prnt == False and injector_ready:
                i_time.append(injection(nct_vlm))
                print(i_time)
                num_inject = num_inject + 1 
                injector_ready = False
                
        except KeyboardInterrupt as e:
            exit_text = "User Interruption"
            print(exit_text)
            break
    
    exit_time =  round(t.clock(),3)
    print(exit_time)
    print(f.start_time,v.start_time)

    f.running = False
    v.running = False
    
   # reset and close the nectar port
   
    Nec_ser.dtr = True
    t.sleep(1)
    Nec_ser.dtr = False
    # Close the port
    Nec_ser.close()
    Ifile.close()
    winsound.PlaySound("C:\Users\Toby\Downloads\seaside-01.wav", winsound.SND_ALIAS)
    #print("active threads " + str(threading.active_count())+ "\n")
    f.join()
    v.join()
    
    print("Goodbye!")
