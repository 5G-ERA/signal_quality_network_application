#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from influxdb import InfluxDBClient


def out_5g_signal():

    # Setup database

    #host='localhost'
    #port=8086
    #username='test5g'
    #password='TEST2023'
    #database='openwrt'

    host = rospy.get_param('/color_publisher/host')
    port = rospy.get_param('/color_publisher/port')
    username = rospy.get_param('/color_publisher/username') 
    password = rospy.get_param('/color_publisher/password') 
    database = rospy.get_param('/color_publisher/database') 

    client = InfluxDBClient(host, port, username, password, database)
    
    connection_success = False
    # Checking if connection with database is successful
    if client.switch_database(database):
        rospy.loginfo('Connecting InfluxDB %s to host: "%s"',database, host)        
        connection_success = True
        
    else:
        client.close()
        rospy.loginfo('Connecting InfluxDB %s to host: "%s"',database, host)
        print('Connecting InfluxDB "'+ database +'" to host: "' + host + '"')
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