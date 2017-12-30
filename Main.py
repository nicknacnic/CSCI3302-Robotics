#!/usr/bin/env python
import serial
import string
import cv2
import numpy as np
import time
from nltk.tokenize import word_tokenize
from nltk.corpus import stopwords
from nltk.stem import PorterStemmer

def sendData(indexNumber):
    arduinoData = serial.Serial("/dev/cu.usbmodem1411",9600)
    arduinoData.write(indexNumber)
def parseObject(colorCode):
    # Will map pixel range to a index number
    # Python3 code - May need to use xrange
    #mapDictionary = {zero: range("Enter pixel range here"),\
                    #one: range("Enter pixel range here"),\
                    #two: range("Enter pixel range here"),\
                    #three: range("Enter pixel range here"),\
                    #four+: range("Enter pixel range here"),\
                    #five: range("Enter pixel range here"),\
                    #six: range("Enter pixel range here"),\
                    #seven: range("Enter pixel range here"),\
                    #eight: range("Enter pixel range here"),\
                    #nine: range("Enter pixel range here"),\
                    #ten: range("Enter pixel range here"),\
                    #eleven: range("Enter pixel range here"),\
                    #twelve: range("Enter pixel range here"),\
                    #thirteen: range("Enter pixel range here"),\
                    #fourteen: range("Enter pixel range here"),\
                    #fifteen: range("Enter pixel range here")}
    indexNumber = None
    cap = cv2.VideoCapture(0)
    time.sleep(1)
    if cap.isOpened():
        while(True):
            ret, frame = cap.read()
            # blurring the frame that's captured
            frame_gau_blur = cv2.GaussianBlur(frame, (3, 3), 0)
            # converting BGR to HSV
            hsv = cv2.cvtColor(frame_gau_blur, cv2.COLOR_BGR2HSV)

            #Blue object acceptable color range
            lower_blue = np.array([110, 50, 50])
            higher_blue = np.array([130, 255, 255])
            blue_range = cv2.inRange(hsv, lower_blue, higher_blue)
            #Red object acceptable color range
            lower_red = np.array([150,150,50])
            higher_red = np.array([180,255,150])
            red_range = cv2.inRange(hsv, lower_red, higher_red)
            #Green object acceptable color range
            lower_green = np.array([50,150,50])
            higher_green = np.array([70,255,150])
            green_range = cv2.inRange(hsv, lower_green, higher_green)

            # Apply color mask
            colorMask = None
            if colorCode == 1:
                mask = cv2.inRange(hsv, lower_red, higher_red)
            elif colorCode == 2:
                mask = cv2.inRange(hsv, lower_green, higher_green)
            elif colorCode == 3:
                mask = cv2.inRange(hsv, lower_blue, higher_blue)
            result = cv2.bitwise_and(frame, frame, mask = mask)

            res_color = cv2.bitwise_and(frame_gau_blur,frame_gau_blur, mask=mask)

            color_and_gray = cv2.cvtColor(res_color, cv2.COLOR_BGR2GRAY)
            canny_edge = cv2.Canny(color_and_gray, 50, 240)
            # applying HoughCircles
            circles = cv2.HoughCircles(canny_edge, cv2.HOUGH_GRADIENT, dp=1, minDist=10,
                param1=10, param2=20, minRadius=25, maxRadius=75)
            cir_cen = []
            if circles != None:
                # circles = np.uint16(np.around(circles))
                for i in circles[0,:]:
                    # drawing on detected circle and its center
                    cv2.circle(frame,(i[0],i[1]),i[2],(0,0,0),2)
                    cv2.circle(frame,(i[0],i[1]),2,(255,255,255),3)
                    cir_cen.append((i[0],i[1]))
            print cir_cen


            # Draw verticle line in frame
            cv2.line(frame,(300,0),(300,720),(255,0,0),3)
            cv2.line(frame,(600,0),(600,720),(255,0,0),3)
            cv2.line(frame,(900,0),(900,720),(255,0,0),3)

            # Draw horizontal line in frame
            cv2.line(frame,(0,180),(1200,180),(255,0,0),3)
            cv2.line(frame,(0,360),(1200,360),(255,0,0),3)
            cv2.line(frame,(0,540),(1200,540),(255,0,0),3)
            print (cir_cen)
            if (len(cir_cen)!=0):
                if cir_cen[0][0]<300:
                    if cir_cen[0][1]<180:
                        sendData('twelve')
                    elif cir_cen[1]>180 and cir_cen[0][1]<360:
                        sendData('eight')
                    elif cir_cen[0][1]>360 and cir_cen[0][1]<540:
                        sendData('four+')
                    elif cir_cen[0][1]>540:
                        sendData('zero')
                if cir_cen[0][0]>300 and cir_cen[0][0]<600:
                    if cir_cen[0][1]<180:
                        sendData('thirteen')
                    elif cir_cen[0][1]>180 and cir_cen[0][1]<360:
                        sendData('nine')
                    elif cir_cen[0][1]>360 and cir_cen[0][1]<540:
                        sendData('five')
                    elif cir_cen[0][1]>540:
                        sendData('one')
                elif cir_cen[0][0]>600 and cir_cen[0][0]<900:
                    if cir_cen[0][1]<180:
                        sendData('fourteen')
                    elif cir_cen[0][1]>180 and cir_cen[0][1]<360:
                        sendData('ten')
                    elif cir_cen[0][1]>360 and cir_cen[0][1]<540:
                        sendData('six')
                    elif cir_cen[0][1]>540:
                        sendData('two')
                elif cir_cen[0][0]>900:
                    if cir_cen[0][1]<180:
                        sendData('fifteen')
                    elif cir_cen[0][1]>180 and cir_cen[0][1]<360:
                        sendData('eleven')
                    elif cir_cen[0][1]>360 and cir_cen[0][1]<540:
                        sendData('seven')
                    elif cir_cen[0][1]>540:
                        sendData('three')
            cv2.imshow('circles', frame)
            cv2.imshow('gray', color_and_gray)
            cv2.imshow('canny', canny_edge)
            k = cv2.waitKey(5) & 0xFF
            if k == 27:
                break
        cv2.destroyAllWindows()
        time.sleep(1)
    else:
        print 'no cam'



def showCamera():
    cap = cv2.VideoCapture(0)
    time.sleep(1)
    if cap.isOpened():
        while(True):
            ret, frame = cap.read()
            # blurring the frame that's captured
            frame_gau_blur = cv2.GaussianBlur(frame, (3, 3), 0)
            # converting BGR to HSV
            hsv = cv2.cvtColor(frame_gau_blur, cv2.COLOR_BGR2HSV)

            #Blue object acceptable color range
            lower_blue = np.array([110, 50, 50])
            higher_blue = np.array([130, 255, 255])
            blue_range = cv2.inRange(hsv, lower_blue, higher_blue)
            #Red object acceptable color range
            lower_red = np.array([150,150,50])
            higher_red = np.array([180,255,150])
            red_range = cv2.inRange(hsv, lower_red, higher_red)
            #Green object acceptable color range
            lower_green = np.array([50,150,50])
            higher_green = np.array([70,255,150])
            green_range = cv2.inRange(hsv, lower_green, higher_green)
            # Combine masks
            mask = blue_range + red_range + green_range
            # Draw verticle line in frame
            cv2.line(frame,(300,0),(300,720),(255,0,0),3)
            cv2.line(frame,(600,0),(600,720),(255,0,0),3)
            cv2.line(frame,(900,0),(900,720),(255,0,0),3)

            # Draw horizontal line in frame
            cv2.line(frame,(0,180),(1200,180),(255,0,0),3)
            cv2.line(frame,(0,360),(1200,360),(255,0,0),3)
            cv2.line(frame,(0,540),(1200,540),(255,0,0),3)

            res_color = cv2.bitwise_and(frame_gau_blur,frame_gau_blur, mask=mask)
            color_and_gray = cv2.cvtColor(res_color, cv2.COLOR_BGR2GRAY)
            canny_edge = cv2.Canny(color_and_gray, 50, 240)
            # applying HoughCircles
            circles = cv2.HoughCircles(canny_edge, cv2.HOUGH_GRADIENT, dp=1, minDist=10,
                param1=10, param2=20, minRadius=100, maxRadius=120)
            cir_cen = []
            if circles != None:
                # circles = np.uint16(np.around(circles))
                for i in circles[0,:]:
                    # drawing on detected circle and its center
                    cv2.circle(frame,(i[0],i[1]),i[2],(0,255,0),2)
                    cv2.circle(frame,(i[0],i[1]),2,(0,0,255),3)
                    cir_cen.append((i[0],i[1]))
            print cir_cen
            cv2.imshow('circles', frame)
            cv2.imshow('gray', color_and_gray)
            cv2.imshow('canny', canny_edge)
            k = cv2.waitKey(5) & 0xFF
            if k == 27:
                break
        cv2.destroyAllWindows()
        time.sleep(1)
    else:
        print 'no cam'


# NLTK help by: http://textminingonline.com/dive-into-nltk-part-ii-sentence-tokenize-and-word-tokenize
# and: https://github.com/bonzanini/nlp-tutorial

def parseLang(inputString):
    color = 0; #initialize return parameter locally
    sent_tokenize_list = word_tokenize(inputString) #split string into tokens (words)
    all_tokens_lower = [t.lower() for t in sent_tokenize_list] #lower case for all tokens
    stop_list = stopwords.words('english') + list(string.punctuation) #remove pronouns, conjunctions, articles, conjunctions, etc.
    tokens_no_stop = [token for token in all_tokens_lower if token not in stop_list] #final sterilized token list
    if any("red" in s for s in tokens_no_stop):
        color = 1
    elif any("green" in s for s in tokens_no_stop):
        color = 2
    elif any("blue" in s for s in tokens_no_stop):
        color = 3
    return color;



def sparkiMenu():
    print("What would you like to do?\n")
    print("1. Tell sparki which color block to retrieve(Currently only red, blue, green compatible).")
    print("2. Tell sparki which index to go to.")
    print("3. Show cameras.")
    print("4. Quit the program.")
    menuSelect = input("")

    try:
        # Send Sparki to the object
        if(int(menuSelect) == 1):
            inputString = raw_input("Tell sparki what to do: (e.g. 'Retrieve the red ball.')\n\n")
            color = parseLang(inputString)
            if color == 0:
                print ("Error reading input, please try again")
            elif color == 1:
                print ("Retrieving *red* object...")
                parseObject(1)
            elif color == 2:
                print ("Retrieving *green* object...")
                parseObject(2)
            elif color == 3:
                print ("Retrieving *blue* object...")
                parseObject(3)
            return 1
        # Send sparki to an index
        elif(int(menuSelect) == 2):
            indexLocation = int(raw_input("Which index would you like Sparki to go to?\n\n"))
            if indexLocation == 0:
                print("Sparki is at index 0")
            elif indexLocation ==1:
                sendData('one')
            elif indexLocation ==2:
                sendData('two')
            elif indexLocation ==3:
                sendData('three')
            elif indexLocation ==4:
                sendData('four+')
            elif indexLocation ==5:
                sendData('five')
            elif indexLocation ==6:
                sendData('six')
            elif indexLocation ==7:
                sendData('seven')
            elif indexLocation ==8:
                sendData('eight')
            elif indexLocation ==9:
                sendData('nine')
            elif indexLocation ==10:
                sendData('ten')
            elif indexLocation ==11:
                sendData('eleven')
            elif indexLocation ==12:
                sendData('twelve')
            elif indexLocation ==13:
                sendData('thirteen')
            elif indexLocation ==14:
                sendData('fourteen')
            elif indexLocation ==15:
                sendData('fifteen')
            else:
                print("Incorrect value, please try again")
            return 1
        #Show the cameras
        elif(int(menuSelect) == 3):
            showCamera()
            return 1
        #Quit the program
        elif(int(menuSelect) == 4):
            return 0
        else:
            print("Invalid choice.")
            return 1
    except ValueError:
        print("Invalid value choice.")
        return 1





if __name__ == "__main__":
    print ("Welcome to Roy, Zijun, and Nic's Final project!\n\n")
    print ("You will be able to tell sparki which colored balls to retrieve.\n\n")
    print ("This project utiliizes NLTK and openCV.\n\n")
    print ("This is for Brad Hayes' CSCI 3302 Intro to Robotics, Fall '17'\n\n")
    IOswitch = True
    while(IOswitch == True):
        IOswitch = sparkiMenu()
    print("Thank you for a great semester!")
