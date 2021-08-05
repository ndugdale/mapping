#!/usr/bin/env
import rospy
import rospkg
import rosbag
import os
import pandas as pd

import std_msgs.msg
import geometry_msgs.msg
import tf2_msgs.msg
import tf_conversions
import sensor_msgs.msg

import numpy as np
import ros_numpy as ros_np
import math

def cone_model(sensor_range,angle,point_count):
    '''
    Outputs a list of points to represent sonar readings according to a cone model
    Inputs:
    - range: sonar range in m
    - angle: beam angle in radians
    - point_count: the number of (x,y,z) points along the diameter of the circle formed by
        the outputted points
    Outputs:
    - points: list of tuples in the form [(x1,y1,z1),(x2,y2,z2),...]
    '''
    radius = sensor_range*math.tan(angle/2)
    dim = np.linspace(-radius,radius,point_count)
 
    points = []
    for z in dim:
        for y in dim:
            # if point is in the cone boundary, append it to the list
            if z*z+y*y <= radius:
                points.append((sensor_range,y,z))

    return points

def range_to_cone_pointCloud2(sensor_range,beam_angle,point_count,stamp,frame):
    pts_list = cone_model(sensor_range,beam_angle,point_count)    # cone model            
    pts = np.array([pts_list], dtype=[('x','<f4'),('y','<f4'),('z','<f4')])
    msg = ros_np.point_cloud2.array_to_pointcloud2(
            pts,
            stamp = stamp,
            frame_id = frame
        )
    return msg

def tf_static_msg(stamp,header_frame,child_frame,x,y,z,roll,pitch,yaw):
    t_s = geometry_msgs.msg.TransformStamped()
    t_s.header.stamp = stamp
    t_s.header.frame_id = header_frame
    t_s.child_frame_id = child_frame
    t_s.transform.translation.x = x
    t_s.transform.translation.y = y
    t_s.transform.translation.z = z
    q_s = tf_conversions.transformations.quaternion_from_euler(roll,pitch,yaw)
    t_s.transform.rotation.x = q_s[0]
    t_s.transform.rotation.y = q_s[1]
    t_s.transform.rotation.z = q_s[2]
    t_s.transform.rotation.w = q_s[3]
    tfm_s = tf2_msgs.msg.TFMessage([t_s])

    return tfm_s


def generate_rosbags():

    # initialise node 
    rospy.init_node('rosbag_from_csv')
    rate = rospy.Rate(10)
    rospy.sleep(1)

    print('\nGenerating rosbag...\n')

    if not rospy.is_shutdown():

        data_name_list =[
            'r20210612_231302_SS02_depth_servo',                    # 0
            'r20210613_045224_SS03_TF_NPZ_04',                      # 1
            'r20210613_050724_SS03_TF_NPZ_04',                      # 2
            'r20210613_055428_SS03_TF_NPZ_04',                      # 3
            'r20210613_064305_SS03_TF_NPZ_04',                      # 4
            'r20210613_233359_SS04_TF_NPZ_07',                      # 5
            'r20210614_000137_SS04_TF_NPZ_07',                      # 6
            'r20210614_034824_SS05_TF_NPZ_01',                      # 7
            'r20210614_062740_SS06_TF_NPZ_02',                      # 8
            'r20210614_222648_SS07_TF_NPZ_06',                      # 9
            'r20210615_004541_SS08_TF_NPZ_06_second_half',          # 10
            'r20210615_024126_SS09_TF_NPZ_03',                      # 11
            'r20210615_054656_SS10_TF_NPZ_08',                      # 12
            'r20210615_222253_SS11_TF_Ref_N_1',                     # 13
            'r20210616_004810_SS12_TF_Ref_N_1_second_half',         # 14
            'r20210616_023257_SS13_TF_MUZ_01',                      # 15
            'r20210616_050126_SS14_TF_MUZ_02',                      # 16
            'r20210616_215942_SS15_TF_Ref_C_1',                     # 17
            'r20210616_231740_SS16_TF_Ref_C_1_second_half',         # 18
            'r20210616_234712_SS16_TF_Ref_C_1_second_half',         # 19
            'r20210617_010409_SS17_TF_Ref_C_1_third',               # 20
            'r20210617_033152_SS18_TF_NPZ_04_reversed',             # 21
            'r20210617_050102_SS19_TF_NPZ_04_reversed_second_half', # 22
            'scott_reef'                                            # 23    for this dataset we will shift the x,y data by -1000 to ensure they remain in OctoMap bounds
        ]

        # select dataset of interest
        data_name = data_name_list[0]

        # flags for csv file existence
        nav = True
        oas = True
        rdi = True

        # establish file names and paths
        rospack = rospkg.RosPack()
        nav_fname = data_name + '_nav.csv'
        nav_fpath = os.path.join(rospack.get_path('mapping'),'csv',nav_fname)
        oas_fname = data_name + '_oas.csv'
        oas_fpath = os.path.join(rospack.get_path('mapping'),'csv',oas_fname)
        rdi_fname = data_name + '_rdi.csv'
        rdi_fpath = os.path.join(rospack.get_path('mapping'),'csv',rdi_fname)
        bag_fname = data_name + '.bag'

        # create pandas dataframes of csv files
        # if no csv file found, set corresponding flag to zero
        '''
        commas, hashes, and additional comments removed from column headers
        '''
        try:
            nav_f = open(nav_fpath,'r')
            nav_line1 = nav_f.readline()
            nav_df = pd.read_csv(nav_f,delim_whitespace=True,names=nav_line1.replace('#','').replace(',','').split())
        except:
            nav = False
        try:
            oas_f = open(oas_fpath,'r')
            oas_line1 = oas_f.readline()
            oas_df = pd.read_csv(oas_f,delim_whitespace=True,names=oas_line1.replace('#','').replace(',','').replace(
                    '(radians =pi is forward >pi is down)','').replace(
                    '(negative if no return detected)',''
                ).split())
        except:
            oas = False
        rdi_f = open(rdi_fpath,'r')
        try:
            rdi_line1 = rdi_f.readline()
            rdi_df = pd.read_csv(rdi_f,delim_whitespace=True,names=rdi_line1.replace('#','').replace(',','').split())
        except:
            rdi = False
        
        # keep track of the latest simulation time - after this time, a message will be published to save the octomap
        sim_time_end = rospy.Time(0)

        # create/write rosbag
        with rosbag.Bag(os.path.join(rospack.get_path('mapping'),'rosbags',bag_fname),'w') as bag:
            # iterate through nav csv
            if nav == True:
                for row in range(nav_df.shape[0]):

                    # unpack nav dataframe
                    nav_stamp = rospy.Time.from_sec(nav_df['time'][row])
                    if nav_stamp.to_sec() > sim_time_end.to_sec():
                        sim_time_end = nav_stamp
                    if row==0:
                        sim_time_start = nav_stamp

                    y = nav_df['north'][row]                    # NB: x/y data is swapped for Sirius
                    x = nav_df['east'][row]                     # NB: x/y data is swapped for Sirius
                    z = nav_df['depth'][row]             
                    roll = math.radians(nav_df['roll'][row])             
                    pitch = math.radians(nav_df['pitch'][row])    
                    yaw = math.radians(nav_df['yaw'][row])            

                    # write map -> AUV tf to bag
                    t_nav = geometry_msgs.msg.TransformStamped()
                    t_nav.header.stamp = nav_stamp
                    t_nav.header.frame_id = '/mission'
                    t_nav.child_frame_id = '/AUV'
                    t_nav.transform.translation.x = x
                    t_nav.transform.translation.y = y
                    t_nav.transform.translation.z = z-50       # TO FIX (this hopefully brings z-values to +-100 range)

                    if data_name == 'scott_reef':
                        t_nav.transform.translation.x -= 1000
                        t_nav.transform.translation.y -= 1000
                    '''
                    -50 required due to apparent cap on pointcloud_[min|max]_z values to -/+100
                    'scott_reef' dataset exceeds the approx +/-3.2km from map origin boundary that exists
                        for 0.1m resolution, and therefore we manually offset xy vals by -1000
                    '''    
                    q_nav = tf_conversions.transformations.quaternion_from_euler(roll,pitch,yaw)
                    t_nav.transform.rotation.x = q_nav[0]
                    t_nav.transform.rotation.y = q_nav[1]
                    t_nav.transform.rotation.z = q_nav[2]
                    t_nav.transform.rotation.w = q_nav[3]

                    tfm_nav = tf2_msgs.msg.TFMessage([t_nav])
                    bag.write('/tf',tfm_nav,nav_stamp)
            
            # iterate through oas csv
            if oas == True:
                for row in range(oas_df.shape[0]):

                    # unpack oas dataframe
                    oas_stamp = rospy.Time.from_sec(oas_df['time'][row])
                    sonar_pitch = -(oas_df['angle'][row]-math.pi)           # note pi:forwards, >pi:downwards (in map frame)
                    sonar_range = oas_df['range'][row]

                    # only include data that recorded a finite range measurement
                    ''' 
                    currently there is no distinction between measurements that:
                    - are not in range (i.e. far away)
                    - have been flagged due to the sonar measuring a part of the AUV
                    '''
                    if sonar_range > 0:
                        if oas_stamp.to_sec() > sim_time_end.to_sec():
                            sim_time_end = oas_stamp
                        if row == 0 and oas_stamp.to_sec()<sim_time_start.to_sec():
                            sim_time_start = oas_stamp

                        # write AUV -> sonar tf to bag
                        t_oas = geometry_msgs.msg.TransformStamped()
                        t_oas.header.stamp = oas_stamp
                        t_oas.header.frame_id = '/AUV'
                        t_oas.child_frame_id = '/sonar'
                        t_oas.transform.translation.x = 0.0
                        t_oas.transform.translation.y = 0.0
                        t_oas.transform.translation.z = -0.75   # ESTIMATE: oas sonar is approx 0.75m above dvl
                        q_oas = tf_conversions.transformations.quaternion_from_euler(0,sonar_pitch,0)
                        t_oas.transform.rotation.x = q_oas[0]
                        t_oas.transform.rotation.y = q_oas[1]
                        t_oas.transform.rotation.z = q_oas[2]
                        t_oas.transform.rotation.w = q_oas[3]

                        tfm_oas = tf2_msgs.msg.TFMessage([t_oas])
                        bag.write('/tf',tfm_oas,oas_stamp)

                        # write sonar pointCloud2 message to bag
                        # sonar_pts_list = [(sonar_range,0,0)]                    # straight line model
                        # cone model parameters
                        sonar_beam_angle = math.radians(7)
                        sonar_point_count = 15
                        sonar_msg = range_to_cone_pointCloud2(sonar_range,sonar_beam_angle,sonar_point_count,oas_stamp,'/sonar')
                        bag.write('/cloud',sonar_msg,oas_stamp)

            # iterate through rdi csv
            if rdi == True:
                for row in range(rdi_df.shape[0]):

                 # unpack rdi dataframe
                    rdi_stamp = rospy.Time.from_sec(rdi_df['time'][row])
                    r_alt = rdi_df['altitude'][row]
                    r1 = rdi_df['r1'][row]
                    r2 = rdi_df['r2'][row]
                    r3 = rdi_df['r3'][row]
                    r4 = rdi_df['r4'][row]

                    if r1>0 or r2>0 or r3>0 or r4>0 or r_alt>0:
                        if rdi_stamp.to_sec() > sim_time_end.to_sec():
                            sim_time_end = rdi_stamp
                        if row == 0 and rdi_stamp.to_sec()<sim_time_start.to_sec():
                            sim_time_start = rdi_stamp

                    # generate rdi pointCloud2 message for non-zero readings and write to bag
                    # note that r1,r2,r3,r4 are the vertical components of the true ranges
                    # cone model parameters
                    dvl_beam_angle = math.radians(7)
                    dvl_point_count = 15
                    if r1>0:
                        r1_msg = range_to_cone_pointCloud2(r1/(math.cos(math.radians(30))),dvl_beam_angle,dvl_point_count,rdi_stamp,'/r1')
                        bag.write('/cloud',r1_msg,rdi_stamp)
                    if r2>0:
                        r2_msg = range_to_cone_pointCloud2(r2/(math.cos(math.radians(30))),dvl_beam_angle,dvl_point_count,rdi_stamp,'/r2')
                        bag.write('/cloud',r2_msg,rdi_stamp)
                    if r3>0:
                        r3_msg = range_to_cone_pointCloud2(r3/(math.cos(math.radians(30))),dvl_beam_angle,dvl_point_count,rdi_stamp,'/r3')
                        bag.write('/cloud',r3_msg,rdi_stamp)
                    if r4>0:
                        r4_msg = range_to_cone_pointCloud2(r4/(math.cos(math.radians(30))),dvl_beam_angle,dvl_point_count,rdi_stamp,'/r4')
                        bag.write('/cloud',r4_msg,rdi_stamp)
                    if r_alt>0:
                        r_alt_msg = range_to_cone_pointCloud2(r_alt,dvl_beam_angle,dvl_point_count,rdi_stamp,'/r_alt')
                        bag.write('/cloud',r_alt_msg,rdi_stamp)
         
            
            # write map -> mission static_tf to bag
            tfm_s = tf_static_msg(sim_time_start,'/map','/mission',0,0,0,math.radians(180),0,0)
            bag.write('/tf_static',tfm_s,sim_time_start)

            # write AUV -> r_alt static_tf to bag
            tfm_s = tf_static_msg(sim_time_start,'/AUV','/r_alt',0,0,0,0,math.radians(-90),0)
            bag.write('/tf_static',tfm_s,sim_time_start)
            # for now we assume r1 (NE), r2 (SE), r3 (SW), r4 (NW)
            # write AUV -> r1 static_tf to bag
            tfm_s = tf_static_msg(sim_time_start,'/AUV','/r1',0,0,0,0,math.radians(-60),math.radians(45))
            bag.write('/tf_static',tfm_s,sim_time_start)
            # write AUV -> r2 static_tf to bag
            tfm_s = tf_static_msg(sim_time_start,'/AUV','/r2',0,0,0,0,math.radians(-60),math.radians(135))
            bag.write('/tf_static',tfm_s,sim_time_start)
            # write AUV -> r3 static_tf to bag
            tfm_s = tf_static_msg(sim_time_start,'/AUV','/r3',0,0,0,0,math.radians(-60),math.radians(-135))
            bag.write('/tf_static',tfm_s,sim_time_start)
            # write AUV -> r4 static_tf to bag
            tfm_s = tf_static_msg(sim_time_start,'/AUV','/r4',0,0,0,0,math.radians(-60),math.radians(-45))
            bag.write('/tf_static',tfm_s,sim_time_start)

        

            # write save message
            save_msg = std_msgs.msg.String()
            save_msg.data = data_name
            bag.write('/save',save_msg,sim_time_end)

        # print completion message
        print('Rosbag successfully written to:\n',os.path.join(rospack.get_path('mapping'),'rosbags',bag_fname))
    
    else:
        rate.sleep()


if __name__ == '__main__':
    try:
        generate_rosbags()
   
    except rospy.ROSInterruptException:
        pass