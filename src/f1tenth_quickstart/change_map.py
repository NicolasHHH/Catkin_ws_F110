#!/usr/bin/env python

# file to change the map of the simulation

import argparse
import os.path
from shutil import copyfile
import yaml
import xml.etree.ElementTree as ET

parser = argparse.ArgumentParser(description='Change the map of the simulation in f1tenth_gym')
parser.add_argument('map', help='Image of the map. The image has to be in the same directory as the corresponding YAML file.')
parser.add_argument('--dir', '-d', dest='dir_target', default='src/f1tenth_gym_ros/', const='src/f1tenth_gym_ros/', nargs='?', help='path to the target directory f1tenth_gym_ros')

args = parser.parse_args()

#print args


# check target directory
if not os.path.exists(args.dir_target):
    print 'ERROR: directory {} not found'.format(args.dir_target)
    exit(1)

dir_target1 = args.dir_target + "maps/"
dir_target2 = args.dir_target + "f1tenth_gym/maps/"
# check target directories
if not os.path.exists(dir_target1):
    print 'ERROR: directory {} not found'.format(dir_target1)
    exit(1)

if not os.path.exists(dir_target2):
    print 'ERROR: directory {} not found'.format(dir_target2)
    print 'Did you execute "build_docker.sh"?'
    exit(1)

dirname = os.path.dirname(args.map)
dirname += '/'
#print 'dirname = {}'.format(dirname)

image_file = os.path.basename(args.map)
basename, extension = os.path.splitext(image_file)
#print 'basename = {}'.format(basename)
#print 'extension = {}'.format(extension)

yaml_file = basename + '.yaml'
#print 'image = {}'.format(image_file)
#print 'yaml = {}'.format(yaml_file)

global_image_file = dirname + image_file
global_yaml_file = dirname + yaml_file
# check existence of the image and YAML file
if not os.path.isfile(global_image_file):
    print 'File {} not found'.format(global_image_file)
    exit(1)

if not os.path.isfile(global_yaml_file):
    print 'File {} not found'.format(global_yaml_file)
    exit(1)


########################################
# copy files in target directories

print "Copy files...",

copyfile(global_image_file, dir_target1 + image_file)
copyfile(global_yaml_file, dir_target1 + yaml_file)

copyfile(global_image_file, dir_target2 + image_file)
copyfile(global_yaml_file, dir_target2 + yaml_file)

print " [DONE]"

########################################
# modify params.yaml

print "Modify params.yaml...",

params_file = args.dir_target + 'params.yaml'
params_stream = file(params_file,'r')
data = yaml.safe_load(params_stream)
params_stream.close()

data['map_path'] = '/f1tenth_gym/maps/' + yaml_file
data['map_img_ext'] = extension

params_stream = file(params_file,'w')
yaml.dump(data, params_stream)

print " [DONE]"

########################################
# modify gym_bridge.launch

print "Modify gym_bridge.launch...",

bridge_file = args.dir_target + 'launch/gym_bridge.launch'
#print "bridge_file = {}".format(bridge_file)
tree = ET.parse(bridge_file)
root = tree.getroot()
for arg in root.iter('arg'):
    if 'map' == arg.get('name'):
        arg.set('default','$(find f1tenth_gym_ros)/maps/' + yaml_file)
tree.write(bridge_file)

print " [DONE]"
