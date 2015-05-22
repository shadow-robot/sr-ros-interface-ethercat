#!/usr/bin/env python

import sys
import rospy
import rosparam
import rospkg
from os import path, listdir, system, walk
import subprocess
import argparse


def find_location(search_item, item_type, search_path='/'):
    # search for location of a file or directory, not in ros path, returns path
    for root, dirname, filename in walk(''.join(search_path)):
        if item_type == 'd':
            if search_item in dirname:
                location = path.join(root, search_item) + '/'
                return location
        elif item_type == 'f':
            if search_item in filename:
                location = path.join(root, search_item)
                return location


def load_parameters(params, config_file='y'):
    # load parameters from configuration yaml into parameter server
    if config_file == 'y':
        paramlist=rosparam.load_file(params)
        for params, ns in paramlist:
            rosparam.upload_params(ns,params)
    else:
        rospy.set_param(params[0], params[1])


def replace_string(file_list, replacements, w='n'):
    # replace string, written to file or output returned
    for i in file_list:
        for r in replacements.keys():
            template_file = open(i, 'r')
            file_data = template_file.read()
            template_file.close()
            new_data = file_data.replace(r, replacements[r])
            if w == 'y':
                f = open(i, 'w')
                f.write(new_data)
                f.close()
            else:
                return new_data


def main():
    rospy.init_node('configuration_load')

    parser = argparse.ArgumentParser()
    parser.add_argument('--hand', dest='hand')
    parser.add_argument('--install_path', dest='install_dir')
    parser.add_argument('--debug', dest='debug')
    parser.add_argument('--eth', dest='eth_port')
    args, unknown = parser.parse_known_args()

    debug = args.debug
    hand = str(args.hand).lower()
    install_dir = args.install_dir.rstrip('/')
    cfg = 'configuration.yaml'

    # check if installation directory is in ros path
    ros_paths = rospkg.get_ros_paths()

    # find installation directory and load configuration.yaml
    if hand == 'bimanual':
        if install_dir in ros_paths:
            rospy.logerr("Cannot launch bimanual from default sr-config location in ROS path")
            sys.exit(0)
        else:
            hand_dict = {'hand': ['right', 'left'], 'prefix': ['rh', 'lh'], 'conf_dir': []}
            for h in hand_dict['hand']:
                conf_dir = path.join(install_dir, h, 'sr-config')
                hand_dict['conf_dir'].append(conf_dir)
                cfg_loc = find_location(cfg, 'f', conf_dir)
                load_parameters(cfg_loc)
    else:
        hand_dict = {'hand': [hand], 'prefix': [], 'conf_dir': []}
        if install_dir in ros_paths:
            hand_dict['conf_dir'].append(path.join(install_dir, 'sr-config'))
        else:
            hand_dict['conf_dir'].append(path.join(install_dir, ''.join(hand_dict['hand']), 'sr-config'))

        if hand == 'right':
            hand_dict['prefix'].append('rh')
        else:
            hand_dict['prefix'].append('lh')
        cfg_loc = find_location(cfg, 'f', hand_dict['conf_dir'])
        load_parameters(cfg_loc)

    # find 'host' directory and create right or left specific files
    for i in range(len(hand_dict['hand'])):
        c, h, p = [hand_dict[j][i] for j in sorted(hand_dict.keys())]
        host = find_location('host', 'd', c)
        file_list = listdir(host)
        host_conf_files = []
        for l in file_list:
            host_conf_files.append(host + l)
        replace_string(host_conf_files, {'_lh_': '_' + p + '_', '_rh_': '_' + p + '_',
                                         ' rh': ' ' + p, ' lh': ' ' + p}, 'y')

        # load pwm_control_mode to parameter server
        pwm_control = str(rospy.get_param('pwm_control')).lower()
        if pwm_control == 'true':
            load_parameters(['realtime_loop/' + p + '/default_control_mode', 'PWM'], 'n')
        else:
            load_parameters(['realtime_loop/' + p + '/default_control_mode', 'FORCE'], 'n')

        # replace prefix in sr_edc.launch
        replace_string([rospkg.RosPack().get_path('sr_edc_launch') + '/sr_edc.launch'],
                       {'_lh_': '_' + p + '_', '_rh_': '_' + p + '_'}, 'y')

        # load hand parameters
        sr_ethercat_dir = find_location('sr_ethercat_hand_config', 'd', c)
        system('roslaunch sr_edc_launch load_hand_parameters.xml hand_id:=' + p + ' sr_ethercat_dir:=' + sr_ethercat_dir
               + ' pwm_control:=' + str(rospy.get_param('pwm_control')) + ' use_ns:=' + str(rospy.get_param('use_ns')))

    # set eth_port, based on argument given or if not, take from parameter server
    eth_port = args.eth_port
    if eth_port == 'none':
        if hand == 'bimanual':
            eth_port = str(rospy.get_param('rh_eth_port')) + '_' + str(rospy.get_param('lh_eth_port'))
        else:
            eth_port = rospy.get_param(str(hand_dict['prefix']) + '_eth_port')

    # find robot description
    robot_description_file = find_location(rospy.get_param('robot_description_file'), 'f',
                                           rospy.get_param('robot_description_path'))
    xacro_file = path.join(rospkg.RosPack().get_path('xacro'), 'xacro.py')

    # load robot description and launch edc launch file
    if hand == 'bimanual':
        robot_description_detail = subprocess.check_output('python ' + xacro_file + " "
                                                           + robot_description_file, shell=True)
        rospy.set_param('robot_description', robot_description_detail)
        system('roslaunch sr_edc_launch sr_edc_bimanual.launch debug:=' + debug + ' eth_port:='+eth_port)
    else:
        robot_description_detail = subprocess.check_output('python ' + xacro_file + " "
                                                           + robot_description_file, shell=True)
        rospy.set_param('robot_description', robot_description_detail)
        system('roslaunch sr_edc_launch sr_edc.launch debug:=' + debug + ' eth_port:=' + eth_port)


if __name__ == "__main__":
    main()