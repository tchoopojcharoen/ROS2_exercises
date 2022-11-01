#!/usr/bin/python3
"""
This program is free software: you can redistribute it and/or modify it 
under the terms of the GNU General Public License as published by the Free Software Foundation, 
either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. 
If not, see <https://www.gnu.org/licenses/>.
"""

# all neccesary libraries
import yaml
import os
from ament_index_python.packages import get_package_share_directory, get_resource
import numpy as np

# compute 3x3 Special Orthogonal matrix based on given angle and axis of rotation
def rotm(angle=0.0,axis='z'):
    c = np.math.cos(angle)
    s = np.math.sin(angle)
    if axis=='x':
        R = np.array([[1,0,0],[0,c,-s],[0,s,c]])
    elif axis=='y':
        R = np.array([[c,0,s],[0,1,0],[-s,0,c]])
    elif axis=='z':
        R = np.array([[c,-s,0],[s,c,0],[0,0,1]])
    else:
        print('axis error')
        R = np.identity(3)
    return R
# compute 4x4 Special Euclidean matrix that represent a rotation about 
# the given axis of rotation 
def rotate(angle=0.0,axis='z'):
    R = rotm(angle,axis)
    temp = np.concatenate((R,np.zeros((1,3))),0)
    H = np.concatenate((temp,np.array([[0,0,0,1]]).T),1)
    return H
# compute 4x4 Special Euclidean matrix that represent a translation along 
# the given axis of translation 
def translate(distance=0.0,axis='z'):
    H = np.identity(4)
    axes = {'x':0,'y':1,'z':2}
    try:
        H[axes[axis]][3] = distance
    except:
        print('axis error')
    return H
# compute Roll, Pitch, and Yaw based on given rotation matrix 
def rotm2rpy(R,gamma=1):
    sc_x = (gamma*R[2][1],gamma*R[2][2])
    sc_y = (-R[2][0],gamma*np.math.sqrt(R[2][1]**2+R[2][2]**2))
    sc_z = (gamma*R[1][0],gamma*R[0][0])
    sc_pairs = [sc_x,sc_y,sc_z]
    #tol = 0.00000000001
    idx = 0
    r = [0.0,0.0,0.0]
    for sc in sc_pairs:
        r[idx] = np.math.atan2(sc[0],sc[1])
        idx = idx + 1
    return r

# Transform a given parameter YAML file and convert to 
# another YAML that is readable by Xacro. 
def DH2Transform(package,folder,filename):
    path = get_package_share_directory(package)
    file_path = os.path.join(path,folder,filename)
    with open(file_path) as f:
        param = yaml.load(f,Loader=yaml.SafeLoader)
    
    Transforms = []
    for dh in param['DH']:
        
        Rx = rotate(dh[0]*np.math.pi/180,'x')
        Tx = translate(dh[1],'x')
        Rz = rotate(dh[2]*np.math.pi/180,'z')
        Tz = translate(dh[3],'z')
        H = Rx@Tx@Rz@Tz
        p = H[0:3,3].tolist()
        R = H[0:3,0:3]
        r = rotm2rpy(R)
        p_str = str(p[0])+' '+str(p[1])+' '+str(p[2])
        r_str = str(r[0])+' '+str(r[1])+' '+str(r[2])
        transform = {'position':p_str,'orientation':r_str}
        Transforms.append(transform)
    param.pop('DH')
    param['transform']=Transforms
    ee_p = param['ee_position']
    ee_r = param['ee_orientation']
    ee_p_str = str(ee_p[0])+' '+str(ee_p[1])+' '+str(ee_p[2])
    ee_r_str = str(ee_r[0]*np.math.pi/180)+' '+str(ee_r[1]*np.math.pi/180)+' '+str(ee_r[2]*np.math.pi/180)
         
    param['ee_orientation'] = ee_r_str
    param['ee_position'] = ee_p_str
    file_path = os.path.join(path,folder,'properties.yaml')
    with open(file_path,'w') as f:
        yaml.dump(param,f)
    return file_path
