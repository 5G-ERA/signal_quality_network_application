#!/usr/bin/env python3

#colour_pub

import rospy
from std_msgs.msg import String
import os

def handle_keyboard():
    publisher = rospy.Publisher('pcl_colour', String, queue_size=10)

    while True:
        print('\n- Publish pcl colour -')
        print('   1. Publish RED')
        print('   2. Publish GREEN')
        print('   3. Publish BLUE')
        print('   99. Exit')

        colour = input('Input: ')
        '''
        print("menu: "+str(menu))
        rospy.loginfo("Publish RED")
        publisher.publish("RED")
        '''
        
        if str(colour) == "1":
            print("RED")
            publisher.publish("RED")
            rospy.loginfo("Publish RED")

        elif str(colour) == "2":
            print("GREEN")
            publisher.publish("GREEN")
            rospy.loginfo("Publish GREEN")

        elif str(colour) == "3":
            print("BLUE")
            publisher.publish("BLUE")
            rospy.loginfo("Publish BLUE")

        elif str(colour) == "99":
            rospy.on_shutdown(handle_keyboard)
            os._exit(1)
        

if __name__ == '__main__':

    rospy.init_node('cloud_coloring', anonymous=True)
    

    try:
        handle_keyboard()
        #th = threading.Thread(target=handle_keyboard, args=(publisher,))
        #th.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        exit(0)

    
