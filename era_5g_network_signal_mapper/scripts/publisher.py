#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from influxdb import InfluxDBClient

def out_5g_signal():

    host = rospy.get_param('/costmap_translate/host') # By default --> -50
    port = rospy.get_param('/costmap_translate/port') # By default --> -50
    username = rospy.get_param('/costmap_translate/username') # By default --> 0.05
    password = rospy.get_param('/costmap_translate/password') # By default --> 1984
    database = rospy.get_param('/costmap_translate/database') # By default --> robot_map

    #Setup database
    client = InfluxDBClient(host, port, username, password, database)
    connection_success = False
    if client.switch_database(database):
        #check if connection with database is successful
        connection_success = True
    else:
        client.close()
        print("Connection Error")

    if connection_success:

        try: 
            result = client.query('SELECT last("RSRP"), last("RSRQ") from gsmctl;')
        
            signal_strength = "RED"
            pub = rospy.Publisher('chatter', String, queue_size=1)
            rospy.init_node('out_5g_signal', anonymous=True)
            rate = rospy.Rate(1) 
            while not rospy.is_shutdown():
                points = result.get_points()
                for item in points:
                    rsrp = (item['last'])
                    rsrq =(item['last_1'])

                # check signal and printing lowest signal between 2 parameters RSRP and RSRQ
                # GREEN Strong Signal
                # YELLOW Good Signal
                # ORANGE Poor Signal
                # RED No Signal
                if  rsrp < -100.0:
                    signal_strength = "RED"            
                elif rsrq < -20.0:
                    signal_strength = "RED"
                elif  (rsrp <= -90.0) & (rsrp > -100.0):
                    signal_strength = "ORANGE"
                elif  (rsrq <= -15.0) & (rsrq > -20.0):
                    signal_strength = "ORANGE"
                elif  (rsrp <= -80.0) & (rsrp > -90.0):
                    signal_strength = "YELLOW"
                elif  (rsrq <= -10.0) & (rsrq > -15.0):
                    signal_strength = "YELLOW"
                elif rsrp > -80.0:
                    signal_strength = "GREEN"
                elif rsrq > -10.0:
                    signal_strength = "GREEN"
                else:
                    signal_strength = "RED"
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