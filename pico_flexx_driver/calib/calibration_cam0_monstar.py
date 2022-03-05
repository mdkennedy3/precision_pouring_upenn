#!/usr/bin/python2

from tf.transformations import *
import math
import yaml
import sys
import argparse

def write_transform(ofname, T_cn_cnm1):
    T = inverse_matrix(T_cn_cnm1);
    yaw, pitch, roll = euler_from_matrix(T, 'rzyx');
    f = open(ofname, 'w');
    f.write('<launch>\n');
    f.write('  <arg name="leftcam_frame"/>\n');
    f.write('  <arg name="monstar_frame"/>\n');
    f.write('  <node pkg="tf2_ros" type="static_transform_publisher" name="leftcam_monstar_tf"\n');
    f.write('      args="%.5f %.5f %.5f   %.6f %.6f %.6f\n' % (T[0][3], T[1][3], T[2][3], yaw, pitch, roll));
    f.write('            $(arg leftcam_frame) $(arg monstar_frame)"/>\n');
    f.write('</launch>\n');
    f.close();
    print 'written transform to file: %s' % (ofname);

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='convert monstar calibration file to static transform.')
    parser.add_argument('ifname', metavar='calib_input_file', default=None, help='calibration input file')
    parser.add_argument('ofname', metavar='static_tf_output_file', default=None, help='static tf output file')

    args = parser.parse_args()
    cname = 'cam1';
    
    with open(args.ifname, 'r') as stream:
        try:
            params = yaml.load(stream);
            if not params[cname]:
                raise ValueError('cannot find cam in calib file!');
            if not params[cname]['T_cn_cnm1']:
                raise ValueError('cannot find cam/T_cn_cnm1 in calib file!');
            write_transform(args.ofname, params[cname]['T_cn_cnm1'])
        except yaml.YAMLError as exc:
            print(exc)
        except ValueError as exc:
            print(exc)
