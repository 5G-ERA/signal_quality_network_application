#!/usr/bin/env python3
# git commit
import rospy
from std_msgs.msg import String
from influxdb import InfluxDBClient

#Setup database
client = InfluxDBClient('192.168.23.245', 8086, 'test5g', 'TEST2023', 'openwrt')
connection_success = False
if client.switch_database('openwrt'):
    #check if connection with database is successful
    connection_success = True
else:
    client.close()
    print("Connection Error")

def out_5g_signal():

    if connection_success:

        try: 
            result = client.query('SELECT last("RSRP"), last("RSRQ") from gsmctl;')
        
            signal_strength = "Red"
            pub = rospy.Publisher('chatter', String, queue_size=1)
            rospy.init_node('out_5g_signal', anonymous=True)
            rate = rospy.Rate(1) 
            while not rospy.is_shutdown():
                points = result.get_points()
                for item in points:
                    rsrp = (item['last'])
                    rsrq =(item['last_1'])

                # check signal and printing lowest signal between 2 parameters RSRP and RSRQ
                # Green Strong Signal
                # Yellow Good Signal
                # Orange Poor Signal
                # Red No Signal
                if  rsrp < -100.0:
                    signal_strength = "Red"            
                elif rsrq < -20.0:
                    signal_strength = "Red"
                elif  (rsrp <= -90.0) & (rsrp > -100.0):
                    signal_strength = "Orange"
                elif  (rsrq <= -15.0) & (rsrq > -20.0):
                    signal_strength = "Orange"
                elif  (rsrp <= -80.0) & (rsrp > -90.0):
                    signal_strength = "Yellow"
                elif  (rsrq <= -10.0) & (rsrq > -15.0):
                    signal_strength = "Yellow"
                elif rsrp > -80.0:
                    signal_strength = "Green"
                elif rsrq > -10.0:
                    signal_strength = "Green"
                else:
                    signal_strength = "Red"
                rospy.loginfo(signal_strength)
                pub.publish(signal_strength)
                rate.sleep()
        except Exception as e:
            print(e)
        finally:
            client.close()


if __name__ == '__main__':
    try:
        out_5g_signal()
    except rospy.ROSInitException:
        pass