# Code modified at IBOIS (2023) from 
#
# original author by Naturalpoint (Copyright © 2018)
#
#Licensed under the Apache License, Version 2.0 (the "License")
#you may not use this file except in compliance with the License.
#You may obtain a copy of the License at
#
#http://www.apache.org/licenses/LICENSE-2.0
#
#Unless required by applicable law or agreed to in writing, software
#distributed under the License is distributed on an "AS IS" BASIS,
#WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#See the License for the specific language governing permissions and
#limitations under the License.


# OptiTrack NatNet direct depacketization sample for Python 3.x
#


# Uses the Python NatNetClient.py library to establish a connection (by creating a NatNetClient),
# and receive data via a NatNet connection and decode it using the NatNetClient library.

#Utility functions

import copy
import hashlib
import random
from pyquaternion import Quaternion
import numpy as np


K_SKIP = [0,0,1]
K_FAIL = [0,1,0]
K_PASS = [1,0,0]

# get_tab_str
# generate a string that takes the nesting level into account
def get_tab_str(tab_str, level):
    out_tab_str=""
    loop_range = range(0,level)
    for _ in loop_range:
        out_tab_str+=tab_str
    return out_tab_str

def add_lists(totals, totals_tmp):
    totals[0]+=totals_tmp[0]
    totals[1]+=totals_tmp[1]
    totals[2]+=totals_tmp[2]
    return totals

def test_hash(test_name, test_hash_str, test_object):
    out_str = test_object.get_as_string()
    out_hash_str=hashlib.sha1(out_str.encode()).hexdigest()
    ret_value=True
    if test_hash_str == out_hash_str:
        print("[PASS]:%s"%test_name)
    else:
        print("[FAIL]:%s test_hash_str != out_hash_str"%test_name)
        print("test_hash_str=%s"%test_hash_str)
        print("out_hash_str=%s"%out_hash_str)
        print("out_str =\n%s"%out_str)
        ret_value=False
    return ret_value


def test_hash2(test_name, test_hash_str, test_object, run_test=True):
    ret_value = K_FAIL
    out_str = "FAIL"
    out_str2=""
    indent_string="       "
    if not run_test:
        ret_value = K_SKIP
        out_str = "SKIP"
    elif test_object == None:
        out_str = "FAIL"
        ret_value = K_FAIL
        out_str2 = "%sERROR: test_object was None"%indent_string
    else:

        if str(type(test_object)) != 'NoneType':
            obj_out_str = test_object.get_as_string()
            obj_out_hash_str=hashlib.sha1(obj_out_str.encode()).hexdigest()

        if test_hash_str == obj_out_hash_str:
            out_str = "PASS"
            ret_value = K_PASS
        else:
            out_str2+="%s%s test_hash_str != out_hash_str\n"%(indent_string,test_name)
            out_str2+="%stest_hash_str=%s\n"%(indent_string,test_hash_str)
            out_str2+="%sobj_out_hash_str=%s\n"%(indent_string,obj_out_hash_str)
            out_str2+="%sobj_out_str =\n%s"%(indent_string,obj_out_str)
            ret_value = K_FAIL
    print("[%s]:%s"%(out_str,test_name))

    if len(out_str2):
        print("%s"%out_str2)
    return ret_value


def get_as_string(input_str):
    type_input_str=str(type(input_str))
    if type_input_str == "<class 'str'>":
        return input_str
    elif type_input_str ==  "<class 'NoneType'>":
        return ""
    elif type_input_str == "<class 'bytes'>":
        return input_str.decode('utf-8')
    else:
        print("type_input_str = %s NOT HANDLED"%type_input_str)
        return input_str


#MoCap Frame Classes
class FramePrefixData:
    def __init__(self, frame_number):
        self.frame_number=frame_number

    def get_as_string(self,tab_str="  ", level = 0):
        out_tab_str = get_tab_str(tab_str, level)
        out_str = "%sFrame #: %3.1d\n"%(out_tab_str,self.frame_number)
        return out_str
    
    def get_frame_number(self):
        return self.frame_number

class MarkerData:
    def __init__(self):
        self.model_name=""
        self.marker_pos_list=[]

    def set_model_name(self, model_name):
        self.model_name = model_name

    def add_pos(self, pos):
        self.marker_pos_list.append(list(copy.deepcopy(pos)))
        return len(self.marker_pos_list)


    def get_num_points(self):
        return len(self.marker_pos_list)


    def get_as_string(self, tab_str="  ", level=0):
        out_tab_str = get_tab_str(tab_str, level)
        out_tab_str2 = get_tab_str(tab_str, level+1)
        out_str=""
        if self.model_name != "":
            out_str+="%sModel Name : %s\n"%(out_tab_str, get_as_string(self.model_name))
        marker_count = len(self.marker_pos_list)
        out_str+="%sMarker Count :%3.1d\n"%(out_tab_str, marker_count)
        for i in range(marker_count):
            pos = self.marker_pos_list[i]
            out_str+="%sMarker %3.1d pos : [%3.2f,%3.2f,%3.2f]\n"%(out_tab_str2,i,pos[0], pos[1], pos[2])
        return out_str

    def get_name(self):
        name_list = []
        for i in range(len(self.marker_pos_list)):
            name = [get_as_string(self.model_name)+"_X_Marker_"+str(i),
                    get_as_string(self.model_name)+"_Y_Marker_"+str(i),
                    get_as_string(self.model_name)+"_Z_Marker_"+str(i)]
            name_list+=name
        return name_list
    
    def get_marker_pos_list(self):
        list_marker_pos = []
        for pos in self.marker_pos_list:
            list_marker_pos+=pos
        return list_marker_pos

class MarkerSetData:
    def __init__(self):
        self.marker_data_list=[]
        self.unlabeled_markers=MarkerData()
        self.unlabeled_markers.set_model_name("")

    def add_marker_data(self, marker_data):
        self.marker_data_list.append(copy.deepcopy(marker_data))
        return len(self.marker_data_list)

    def add_unlabeled_marker(self, pos):
        self.unlabeled_markers.add_pos(pos)

    def get_marker_set_count(self):
        return len(self.marker_data_list)

    def get_unlabeled_marker_count(self):
        return self.unlabeled_markers.get_num_points()
    def get_as_string(self, tab_str="  ", level=0):
        out_tab_str  = get_tab_str(tab_str, level)


        out_str=""

        # Labeled markers count
        marker_data_count = len(self.marker_data_list)
        out_str+= "%sMarker Set Count:%3.1d\n"% (out_tab_str,marker_data_count)
        for marker_data in self.marker_data_list:
            out_str += marker_data.get_as_string(tab_str,level+1)

        # Unlabeled markers count (4 bytes)
        unlabeled_markers_count = self.unlabeled_markers.get_num_points()
        out_str += "%sUnlabeled Markers Count:%3.1d\n"%(out_tab_str, unlabeled_markers_count )
        out_str += self.unlabeled_markers.get_as_string(tab_str,level+1)
        return out_str
    
    def get_markers_pos(self):
        marker_pos_list = []
        for marker in self.marker_data_list:
            marker_pos_list+=marker.get_marker_pos_list()
        return marker_pos_list
    
    def get_names(self):
        marker_name_list = []
        for marker in self.marker_data_list:
            marker_name_list+=marker.get_name()
        return marker_name_list
    
    def get_group_count(self):
        list_count = [0]
        for marker_group in self.marker_data_list:
            marker_count = marker_group.get_num_points()
            list_count+=[list_count[-1]+marker_count]
        return list_count
    


class RigidBodyMarker:
    def __init__(self):
        self.pos = [0.0,0.0,0.0]
        self.id_num = 0
        self.size = 0
        self.error = 0

    def get_as_string(self, tab_str="  ", level=0):
        out_tab_str = get_tab_str(tab_str, level)
        out_str = ""

        out_str += "%sPosition: [%3.2f %3.2f %3.2f]\n"%( out_tab_str, self.pos[0], self.pos[1], self.pos[2] )
        out_str += "%sID      : %3.1d\n"%(out_tab_str, self.id_num)
        out_str += "%sSize    : %3.1d\n"%(out_tab_str, self.size)
        return out_str
    
    def get_pos(self):
        return self.pos

    def get_id(self):
        return self.id_num
    

class RigidBody:
    def __init__(self, new_id, pos, rot):
        self.id_num = new_id
        self.pos=pos
        self.rot=rot
        self.rb_marker_list=[]
        self.tracking_valid = False
        self.error = 0.0

    def add_rigid_body_marker(self, rigid_body_marker):
        self.rb_marker_list.append(copy.deepcopy(rigid_body_marker))
        return len(self.rb_marker_list)


    def get_as_string(self, tab_str=0, level=0):
        out_tab_str = get_tab_str(tab_str, level)
        out_tab_str2 = get_tab_str(tab_str, level+1)

        out_str=""

        # header
        out_str += "%sID            : %3.1d\n"% (out_tab_str, self.id_num)
        # Position and orientation
        out_str += "%sPosition      : [%3.2f, %3.2f, %3.2f]\n"% (out_tab_str, self.pos[0], self.pos[1], self.pos[2] )
        out_str += "%sOrientation   : [%3.2f, %3.2f, %3.2f, %3.2f]\n"% (out_tab_str, self.rot[0], self.rot[1], self.rot[2], self.rot[3] )

        marker_count = len(self.rb_marker_list)
        marker_count_range = range( 0, marker_count )

        # Marker Data
        if marker_count > 0:
            out_str += "%sMarker Count: %3.1d\n"%(out_tab_str, marker_count )
            for i in marker_count_range:
                out_str += "%sMarker %3.1d\n"%(out_tab_str2, i)
                rbmarker = self.rb_marker_list[i]
                out_str += rbmarker.get_as_string(tab_str, level+2)

        out_str += "%sMarker Error  : %3.2f\n"% (out_tab_str, self.error)

        # Valid Tracking
        tf_string = 'False'
        if self.tracking_valid:
            tf_string = 'True'
        out_str += "%sTracking Valid: %s\n"%(out_tab_str, tf_string)

        return out_str
    
    def get_marker_pos_list(self):
        pos_list=[]
        for marker in self.rb_marker_list:
            pos = marker.get_pos()
            pos_list.append(pos)
        return pos_list
    
    def get_marker_id_list(self):
        id_list = []
        for marker in self.rb_marker_list:
            id = marker.get_id()
            id_list.append(id)
        return id_list
    
    def get_pos(self):
        return self.pos
    
    def get_rotation(self):
        return self.rot
    
    def get_id(self):
        id = "MODEL_"+str(self.id_num)
        id_list = [id for i in range(7)] # One rb represent 7 values (X,Y,Z pos, X,Y,Z,Q rot)
        return id_list

class RigidBodyData:
    def __init__(self):
        self.rigid_body_list=[]


    def add_rigid_body(self, rigid_body):
        self.rigid_body_list.append(copy.deepcopy(rigid_body))
        return len(self.rigid_body_list)


    def get_rigid_body_count(self):
        return len(self.rigid_body_list)

    def get_as_string(self, tab_str="  ", level=0):
        out_tab_str = get_tab_str(tab_str, level)
        out_str=""
        rigid_body_count=len(self.rigid_body_list)
        out_str += "%sRigid Body Count: %3.1d\n"%(out_tab_str, rigid_body_count)
        for rigid_body in self.rigid_body_list:
            out_str += rigid_body.get_as_string(tab_str, level+1)
        return out_str
    
    def get_rigid_body_list(self):
        return self.rigid_body_list
    
    def get_ids(self):
        id_list = []
        for rb in self.rigid_body_list:
            id_list+=rb.get_id()
        return id_list

class LabeledMarker:
    def __init__(self, new_id, pos, size=0.0, param = 0, residual=0.0):
        self.id_num=new_id
        self.pos = pos
        self.size = size
        self.param = param
        self.residual = residual
        if str(type(size)) == "<class 'tuple'>":
            self.size=size[0]


    def __decode_marker_id(self):
        model_id = self.id_num >> 16
        marker_id = self.id_num & 0x0000ffff
        return model_id, marker_id

    def __decode_param(self):
        occluded = ( self.param & 0x01 ) != 0
        point_cloud_solved = ( self.param & 0x02 ) != 0
        model_solved = ( self.param & 0x04 ) != 0
        return occluded,point_cloud_solved, model_solved

    def get_as_string(self, tab_str, level):
        out_tab_str = get_tab_str(tab_str, level)
        model_id, marker_id = self.__decode_marker_id()
        out_str = ""
        out_str += "%sID                 : [MarkerID: %3.1d] [ModelID: %3.1d]\n"%(out_tab_str, marker_id,model_id)
        out_str += "%spos                : [%3.2f, %3.2f, %3.2f]\n"%(out_tab_str, self.pos[0],self.pos[1],self.pos[2])
        out_str += "%ssize               : [%3.2f]\n"%(out_tab_str, self.size)

        occluded, point_cloud_solved, model_solved = self.__decode_param()
        out_str += "%soccluded           : [%3.1d]\n"%(out_tab_str, occluded)
        out_str += "%spoint_cloud_solved : [%3.1d]\n"%(out_tab_str, point_cloud_solved)
        out_str += "%smodel_solved       : [%3.1d]\n"%(out_tab_str, model_solved)
        out_str += "%serr                : [%3.2f]\n"%(out_tab_str, self.residual)

        return out_str
       
    def get_marker_pos_list(self):
        return list(self.pos)


class LabeledMarkerData:
    def __init__(self):
        self.labeled_marker_list=[]

    def add_labeled_marker(self, labeled_marker):
        self.labeled_marker_list.append(copy.deepcopy(labeled_marker))
        return len(self.labeled_marker_list)

    def get_labeled_marker_count(self):
        return len(self.labeled_marker_list)

    def get_as_string(self, tab_str = "  ", level = 0):
        out_tab_str = get_tab_str(tab_str, level)
        out_tab_str2 = get_tab_str(tab_str, level+1)
        out_str = ""

        labeled_marker_count = len(self.labeled_marker_list)
        out_str += "%sLabeled Marker Count:%3.1d\n"%(out_tab_str, labeled_marker_count )
        for i in range( 0, labeled_marker_count ):
            out_str += "%sLabeled Marker %3.1d\n"%(out_tab_str2,i)
            labeled_marker = self.labeled_marker_list[i]
            out_str += labeled_marker.get_as_string(tab_str, level+2)
        return out_str
    
    def get_markers_pos(self):
        marker_pos_list = []
        for marker in self.labeled_marker_list:
            marker_pos_list+=marker.get_marker_pos_list()
        return marker_pos_list
       
    def get_ids(self):
        ids = []
        for marker in self.labeled_marker_list:
            ids+=marker.get_id()
        return ids
    
    def get_names(self):
        names = []
        for marker in self.labeled_marker_list:
            names+=marker.get_names()
        return names

class FrameSuffixData:
    def __init__(self):
        self.timecode=-1
        self.timecode_sub=-1
        self.timestamp = -1
        self.stamp_camera_mid_exposure = -1
        self.stamp_data_received = -1
        self.stamp_transmit = -1
        self.param = 0
        self.is_recording = False
        self.tracked_models_changed = True


    def get_as_string(self, tab_str="  ", level=0):
        out_tab_str = get_tab_str(tab_str, level)

        out_str = ""
        if not self.timestamp == -1:
            out_str += "%sTimestamp : %3.2f\n"%(out_tab_str, self.timestamp)
        if not self.stamp_camera_mid_exposure == -1:
            out_str += "%sMid-exposure timestamp : %3.1d\n"%(out_tab_str, self.stamp_camera_mid_exposure)
        if not self.stamp_data_received == -1:
            out_str += "%sCamera data received timestamp : %3.1d\n"%(out_tab_str, self.stamp_data_received)
        if not self.stamp_transmit == -1:
            out_str += "%sTransmit timestamp : %3.1d\n"%(out_tab_str, self.stamp_transmit)

        return out_str

class MoCapData:
    def __init__(self):
        #Packet Parts
        self.prefix_data = None
        self.marker_set_data = None
        self.rigid_body_data = None
        self.skeleton_data = None
        self.labeled_marker_data = None
        self.force_plate_data = None
        self.device_data = None
        self.suffix_data = None
        self.current_frame = None

    def set_prefix_data(self, new_prefix_data):
        self.prefix_data = new_prefix_data

    def set_marker_set_data(self, new_marker_set_data):
        self.marker_set_data = new_marker_set_data

    def set_rigid_body_data(self, new_rigid_body_data):
        self.rigid_body_data = new_rigid_body_data

    def set_skeleton_data(self, new_skeleton_data):
        self.skeleton_data = new_skeleton_data

    def set_labeled_marker_data(self, new_labeled_marker_data):
        self.labeled_marker_data = new_labeled_marker_data

    def set_force_plate_data(self, new_force_plate_data):
        self.force_plate_data = new_force_plate_data

    def set_device_data(self, new_device_data):
        self.device_data = new_device_data

    def set_suffix_data(self, new_suffix_data):
        self.suffix_data = new_suffix_data

    def get_as_string(self, tab_str = "  ", level = 0):
        out_tab_str = get_tab_str(tab_str, level)

        out_str=""
        out_str+= "%sMoCap Frame Begin\n%s-----------------\n"%(out_tab_str,out_tab_str)
        if not self.prefix_data == None:
            out_str+=self.prefix_data.get_as_string()
        else:
            out_str+="%sNo Prefix Data Set\n"%(out_tab_str)

        if not self.marker_set_data == None:
            out_str+=self.marker_set_data.get_as_string(tab_str, level+1)
        else:
            out_str+="%sNo Marker Set Data Set\n"%(out_tab_str)

        if not self.rigid_body_data == None:
            out_str+=self.rigid_body_data.get_as_string(tab_str, level+1)
        else:
            out_str+="%sNo Rigid Body Data Set\n"%(out_tab_str)

        if not self.skeleton_data == None:
            out_str+=self.skeleton_data.get_as_string(tab_str, level+1)
        else:
            out_str+="%sNo Skeleton Data Set\n"%(out_tab_str)

        if not self.labeled_marker_data == None:
            out_str+=self.labeled_marker_data.get_as_string(tab_str, level+1)
        else:
            out_str+="%sNo Labeled Marker Data Set\n"%(out_tab_str)

        if not self.force_plate_data == None:
            out_str+=self.force_plate_data.get_as_string(tab_str, level+1)
        else:
            out_str+="%sNo Force Plate Data Set\n"%(out_tab_str)

        if not self.device_data == None:
            out_str+=self.device_data.get_as_string(tab_str, level+1)
        else:
            out_str+="%sNo Device Data Set\n"%(out_tab_str)

        if not self.suffix_data == None:
            out_str+=self.suffix_data.get_as_string(tab_str, level+1)
        else:
            out_str+="%sNo Suffix Data Set\n"%(out_tab_str)

        out_str+= "%sMoCap Frame End\n%s-----------------\n"%(out_tab_str,out_tab_str)

        return out_str

    # >>> modified >>>>
    def get_as_csv(self, tab_str = "  ", level = 0, frame_number=0, time_stamp=0):
        """ 
            Here is where we sift the data and return it in a csv format.
            The csv format is as follows:
            frame_number, frame_timestamp_in_sec, timber_pos (3 values), camera_pos(4 values in quaternion)

            :param tab_str: the tab string to use as separator
            :param level: the level of indentation
            :param frame_number: the current frame number
            :param time_stamp: the current timestamp of the frame espressed by a float in seconds

            :return: a string containing the csv data
        """
        # get frame number
        _data = [frame_number]

        # timestamp in seconds as a float, example: 1.403636580838555574
        _data.append(time_stamp)

        for rigid_body in self.rigid_body_data.get_rigid_body_list():
            # get position
            rigid_body_pos = list(rigid_body.get_pos())
            _data=_data+rigid_body_pos

            # get orientation
            # Motive output is in (x,y,z,w), for more info docu: https://v22.wiki.optitrack.com/index.php?title=Data_Export:_CSV
            # or https://readthedocs.org/projects/python-natnet/downloads/pdf/latest/:
            """
            In the NatNet data stream, orientation data is represented as a quaternion. Quaternion orientations are order independent,
            however they do indicate a handedness. When decomposing quaternions into Euler angles, it is important to consider the
            coordinate system conventions you want to convert into. An Euler angle convention must account for:
             Rotation order
             Left handed or Right handed
             Static (Global) or Relative (Local) Axes
            As an example, the OptiTrack Motive software uses the following "Motive" coordinate system convention:

            X (Pitch), Y (Yaw), Z (Roll), Right-Handed (RHS), Relative Axes (aka 'local')
            """
            # be sure that the data streaming axis is set to Z-up from the "data streaming pane"
            # # (https://v21.wiki.optitrack.com/index.php?title=Data_Streaming_Pane)
            rigid_body_rot = list(rigid_body.get_rotation())
            _data=_data+rigid_body_rot
        return _data
    
    # >>> modified >>>>
    def get_header(self):
        """ Build the headers of the csv """
        # rigid body names
        rb_ids = self.rigid_body_data.get_ids()
        marker_names = self.marker_set_data.get_names()
        marker_count = self.marker_set_data.get_group_count()
        rigid_body_names = [marker_names[marker_count[i//7]*3][:-11] for i in range(len(rb_ids))]

        rigid_body_names[0] = rigid_body_names[0]+"_posX"
        rigid_body_names[1] = rigid_body_names[1]+"_posY"
        rigid_body_names[2] = rigid_body_names[2]+"_posZ"
        rigid_body_names[3] = rigid_body_names[3]+"_rotX"
        rigid_body_names[4] = rigid_body_names[4]+"_rotY"
        rigid_body_names[5] = rigid_body_names[5]+"_rotZ"
        rigid_body_names[6] = rigid_body_names[6]+"_rotW"

        rigid_body_names[7] = rigid_body_names[7]+"_posX"
        rigid_body_names[8] = rigid_body_names[8]+"_posY"
        rigid_body_names[9] = rigid_body_names[9]+"_posZ"
        rigid_body_names[10] = rigid_body_names[10]+"_rotX"
        rigid_body_names[11] = rigid_body_names[11]+"_rotY"
        rigid_body_names[12] = rigid_body_names[12]+"_rotZ"
        rigid_body_names[13] = rigid_body_names[13]+"_rotW"

        return ["frame_number", "timestamp"] + rigid_body_names