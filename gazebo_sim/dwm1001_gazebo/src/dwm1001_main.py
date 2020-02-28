#!/usr/bin/env python
from dwm1001_systemDefinitions import SYS_DEFS

import rospy, time, serial, os
from dwm1001_apiCommands  import DWM1001_API_COMMANDS
from dwm1001.msg  import anchor
from std_msgs.msg import Int32


rospy.init_node('dwm1001',anonymous = False)
#pub_numbers_of_anchors = rospy.Publisher('/dwm1001/no._anchors')

os.popen("sudo chmod 777 /dev/ttyACM0","w")

rate=rospy.Rate(30)

serialReadLine = ""

serialPortDwm1001 = serial.Serial(
	port = "/dev/ttyACM0",
	baudrate = 115200,
	parity=SYS_DEFS.parity,
	stopbits=SYS_DEFS.stopbits,
	bytesize=SYS_DEFS.bytesize)

no_anchors = 3

class dwm1001_localizer:

    def initializeDWM1001API(self):
		# reset incase previuos run didn't close properly
        serialPortDwm1001.write(DWM1001_API_COMMANDS.RESET)
        # send ENTER two times in order to access api
        serialPortDwm1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
        time.sleep(0.5)
        serialPortDwm1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
        time.sleep(0.5)
        # send a third one - just in case
        serialPortDwm1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)

    def spiltcomma(self,data):
    	data = [x.strip() for x in data.strip().split(',')]
    	return data

    def publishPos(self,data_arr):
    	for network in data_arr:
    	    if 'DIST' in network:
    	        detected = int(data_arr[data_arr.index(network) + 1])
    		anchor_=anchor()
    		for i in range(detected):
    		    id_ = str(data_arr[data_arr.index(network) + 6*i+3])
	    	    anchor_.header.frame_id = '/dwm1001_'+str(id_)
		    anchor_.header.stamp = rospy.Time.now()
		    anchor_.x = float(data_arr[data_arr.index(network) + 6*i +4])
		    anchor_.y = float(data_arr[data_arr.index(network) + 6*i +5])
		    anchor_.z = float(data_arr[data_arr.index(network) + 6*i +6])
		    anchor_.range = float(data_arr[data_arr.index(network) + 6*i + 7])
		    anchor_.device_id = id_

		    anchor_pub = rospy.Publisher("/dwm1001/anchor"+str(id_),anchor, queue_size=3)
		    anchor_pub.publish(anchor_)
	    elif 'POS' in network:
		tag_=anchor()
                id_ = str('5617')
                tag_.header.frame_id = '/dwm1001_'+str(id_)
                tag_.header.stamp = rospy.Time.now()
                tag_.x = float(data_arr[data_arr.index(network) + 1])
                tag_.y = float(data_arr[data_arr.index(network) + 2])
                tag_.z = float(data_arr[data_arr.index(network) + 3])
                tag_.range = float(data_arr[data_arr.index(network) + 4])
                tag_.device_id = id_
                tag_pub = rospy.Publisher("/dwm1001/tag"+str(id_),anchor, queue_size=3)
                tag_pub.publish(tag_)


	
    def main(self):
	global serialReadLine

	serialPortDwm1001.close()
	time.sleep(1)
	serialPortDwm1001.open()

	if(serialPortDwm1001.isOpen()):
	    rospy.loginfo("Port opened: "+ str(serialPortDwm1001.name))
	    self.initializeDWM1001API()
	    time.sleep(2)
	    serialPortDwm1001.write(DWM1001_API_COMMANDS.LEC)
	    serialPortDwm1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
	    rospy.loginfo("Reading dwm coordintaes")
	else:
	    rospy.loginfo("Can't open port: "+str(serialPortDwm1001.name))

	try:
	    while not rospy.is_shutdown():
		serialReadLine=serialPortDwm1001.read_until()
		try:
		    self.publishPos(self.spiltcomma(serialReadLine))
		except IndexError:
		    rospy.logwarn("Found index out of bound")
	except KeyboardInterrupt:
            rospy.loginfo("Quitting DWM1001 Shell Mode and closing port, allow 1 second for UWB recovery")
            serialPortDwm1001.write(DWM1001_API_COMMANDS.RESET)
            serialPortDwm1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)

        finally:
            rospy.loginfo("Quitting, and sending reset command to dev board")
            # serialPortDwm1001.reset_input_buffer()
            serialPortDwm1001.write(DWM1001_API_COMMANDS.RESET)
            serialPortDwm1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
            rate.sleep()
            if "reset" in serialReadLine:
                rospy.loginfo("succesfully closed ")
                serialPortDwm1001.close()

def start():
    dwm1001 = dwm1001_localizer()
    dwm1001.main()

if __name__ == '__main__':
    try:
	start()
	rospy.spin()
    except rospy.ROSInterruptException:
	pass


