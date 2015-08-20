#!/usr/bin/python

import rospy
import rosbag
import sys
import os
import pickle
import csv		#writing CV files.
import os 		#used to get directory for image topics
import roslib 

"""
    This script converts a bag file of joint states to a set of waypoints.
    You need to record a bag file.    
"""
def run():
    if len(sys.argv) != 2:
        sys.stderr.write('[ERROR] This script only takes input bag file as argument.n')
    else:
        inputFileName = sys.argv[1]
        print "[OK] Found bag: %s" % inputFileName

        bag = rosbag.Bag(inputFileName)
        topicList = readBagTopicList(bag)

        while True:
            if len(topicList) == 0:
                print "No topics in list. Exiting"
                break
            selection  = menu(topicList)

            if selection == -92:
                print "[OK] Printing them all"
                for topic in topicList:
                    extract_data(bag, topic, inputFileName)
                break
            elif selection == -45:
                break
            else:
                topic = topicList[selection]
                extract_data(bag, topic, inputFileName)
                topicList.remove(topicList[selection])

        bag.close()

def getHeader(msg):

    #this function makes up the top of the csv file
    msgType = str(type(msg))	
    msgType = msgType[msgType.index('.')+1:]	
    msgType = msgType[:msgType.index('\'')]

    #Add handlers as necessary for each message type:
    		
    if (msgType == '_automatic_painting__Features'):
        headerRow = ["label", "positionX",  "positionY",  "positionZ",  "orientationX" "orientationY", \
        "orientationZ", "orientationW", "pitch", "yaw"]
    else:
        rospy.logerr("Unsupport Message type %s"%msgType)
        sys.exit(2)

    return headerRow


def getColumns(msg):

    #this function gets the data that is necessary for the csv file - one row at a time from the current msg.
    msgType = str(type(msg))	
    msgType = msgType[msgType.index('.')+1:]	
    msgType = msgType[:msgType.index('\'')]	
    index = 0

    # in this order
    joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    columns_ordered = []

    if (msgType == '_sensor_msgs__JointState'):
        for i in range(len(joint_names)):
            for x in range(len(msg.name)):
                if joint_names[i] == msg.name[x]:
                    columns_ordered.append(msg.position[x])
     
    else:
        rospy.logerror("Unexpected error - AGH!")
        sys.exit(2)
    	
    return columns_ordered

def extract_data (bag, topic, inputFileName):
    
    outFile = "output" + topic.replace("/","-") + ".csv"
    print "[OK] Printing %s" % topic
    print "[OK] Output file will be called %s." % outFile

    try:
        fileH = open(outFile, 'wt')
        fileWriter = csv.writer(fileH)
    except:
        rospy.logerr("Error opening specified output file : %s"%outFile)
        sys.exit(2)
    i = 0
    #with rosbag.Bag(outputFileName, 'w', chunk_threshold=100 * 1024 * 1024) as outbag:
    for topic, msg, t in bag.read_messages(topics=topic):
        #pickle.dump(msg,outputFh)
        #headerRow = getHeader(msg)
        columns = getColumns(msg)
    
        #write the columns or image to the file/folder.
        fileWriter.writerow(columns)
        i = i + 1     
      #outputFh.close()
    print "[OK] Writing DONE (%s rows)" % str(i)

def menu (topicList):
    i = 0
    for topic in topicList:
        print '[{0}] {1}'.format(i, topic)
        i = i+1
    if len(topicList) > 1:
        print '[{0}] Extract all'.format(len(topicList))
        print '[{0}] Exit'.format(len(topicList) + 1)
    else:
        print '[{0}] Exit'.format(len(topicList))

    while True:
        print 'Enter a topic number to extract raw data from:'
        selection = raw_input('>>>')
        if int(selection) == len(topicList):
            return -92 # print all
        elif int(selection) == (len(topicList) +1):
            return -45 # exit
        elif (int(selection) < len(topicList)) and (int(selection) >= 0):
            return int(selection)
        else:
            print "[ERROR] Invalid input"

def readBagTopicList(bag):
    print "[OK] Reading topics in this bag. Can take a while.."
    topicList = []
    for topic, msg, t in bag.read_messages():
        if topicList.count(topic) == 0:
            topicList.append (topic)
        
    print '{0} topics found:'.format(len(topicList))
    return topicList

if __name__ == "__main__":
    run()
