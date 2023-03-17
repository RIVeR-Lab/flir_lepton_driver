#!/usr/bin/env python3.8
'''
Description: ROS node which reads from a FLIR Lepton 2.5 Thermal Imaging Module. It publishes the thermal image as a sensor_msgs/Image.msg type message to the /thermal_img topic 
and publishes true temperature values of each pixel in hundreths of a Kelvin (centiKelvin) as a flir_lepton_driver/Tempuratures.msg type message to the /pxl_temps topic
Author: Jamie Flanagan
Credit: contains code directly copied from https://github.com/groupgets/purethermal1-uvc-capture/blob/master/python/uvc-radiometry.py
Created: 3-17-23
Last modified: 3-17-23
'''
import rospy
import cv2
import numpy as np
from uvctypes import *
from queue import Queue
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Header
from flir_lepton_driver.msg import Tempuratures
RECORD = False  # Set to true if you want to record video
BUF_SIZE = 10
q = Queue(BUF_SIZE)
def py_frame_callback(frame, userptr):
    array_pointer = cast(frame.contents.data, POINTER(c_uint16 * (frame.contents.width * frame.contents.height)))
    data = np.frombuffer(array_pointer.contents, dtype=np.dtype(np.uint16)).reshape(frame.contents.height, frame.contents.width)    
    if frame.contents.data_bytes != (2 * frame.contents.width * frame.contents.height):
        return
    if not q.full():
        q.put(data)
PTR_PY_FRAME_CALLBACK = CFUNCTYPE(None, POINTER(uvc_frame), c_void_p)(py_frame_callback)

# converts degrees Fahrenheit to centiKelvin
def degF_2_cK(temp_degF):
    return ((temp_degF - 32) * 5 / 9 + 273.15) * 100

# converts centiKelvin to degrees Fahrenheit
def cK_2_degF(temp_cK):
    return (temp_cK / 100 - 273.15) * 9 / 5 + 32

def raw_to_8bit(data):
    cv2.normalize(data, data, 0, 65535, cv2.NORM_MINMAX)
    np.right_shift(data, 8, data)
    return np.uint8(data)

# Converts the 3D raw image data matrix into a 1D list
def serialize_img(raw_matrix_data,dim=3):
    list_data = list()
    for row in raw_matrix_data:
        for pixel in row:
            if dim<3:
                list_data.append(pixel)
            else:
                for color in pixel:
                    list_data.append(color)  
    return list_data

# Converts a serialized image into a numpy ndarray
def deserialize_img(serial_img,w,h,dim):
    s_img = list(serial_img)
    if dim==3:
        img_array = np.zeros((h,w,int(len(serial_img)/w/h)),dtype=np.uint8)
    if dim==2:
        img_array = np.zeros((h,w),dtype=np.uint8)
    for i in range(len(img_array)):
        for j in range(len(img_array[i])):
            if dim == 3:
                for k in range(len(img_array[i][j])):
                    img_array[i][j][k] = s_img.pop(0)
            else:
                img_array[i][j] = s_img.pop(0)
    return img_array

class LeptonThermalImage:
    def __init__(self):
        self.height = 60
        self.width = 80
        self.encoding = 'bgr8'
        self.id = 0
        self.colorize = rospy.get_param('colorize')
        libuvc.uvc_init(byref(ctx), 0)
        libuvc.uvc_find_device(ctx, byref(dev), PT_USB_VID, PT_USB_PID, 0)
        libuvc.uvc_open(dev, byref(devh))
        frame_formats = uvc_get_frame_formats_by_guid(devh, VS_FMT_GUID_Y16)
        libuvc.uvc_get_stream_ctrl_format_size(devh, byref(ctrl), UVC_FRAME_FORMAT_Y16,frame_formats[0].wWidth, frame_formats[0].wHeight, int(1e7 / frame_formats[0].dwDefaultFrameInterval))
        libuvc.uvc_start_streaming(devh, byref(ctrl), PTR_PY_FRAME_CALLBACK, None, 0)
    
    # Reads an image from the camera and returns an Image object
    def get_img(self):
        raw_img = q.get(True,500)
        temp_msg = Tempuratures(serialize_img(raw_img,2))
        img = raw_to_8bit(raw_img)
        img = cv2.flip(img,0)
        encoding = 'mono8'
        if self.colorize > -1:
            img = cv2.applyColorMap(img,self.colorize)
            encoding = 'bgr8'
        if RECORD:
            out.write(img)
            cv2.imshow('Thermal Image',img)
            cv2.waitKey(1)
        serial_img = serialize_img(img)
        header = Header(self.id,rospy.get_rostime(),'map')
        img_msg = Image(header,60,80,encoding,0,240,serial_img)
        compressed_data = np.array(cv2.imencode('.png',img)[1]).tostring()
        comp_msg = CompressedImage(header,'png',compressed_data)
        return temp_msg, img_msg, comp_msg
    def cleanup(self):
        libuvc.uvc_stop_streaming(devh)
        libuvc.uvc_unref_device(dev)
        libuvc.uvc_exit(ctx)
        if RECORD:
            out.release()
            cv2.destroyAllWindows()

if __name__ == '__main__': 
    ctx = POINTER(uvc_context)()
    dev = POINTER(uvc_device)()
    devh = POINTER(uvc_device_handle)()
    ctrl = uvc_stream_ctrl()
    queue_size = rospy.get_param('queue_size')
    if RECORD:
        cv2.namedWindow('Thermal Image',cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Thermal Image',640,480)
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out = cv2.VideoWriter('thermal_video.avi',fourcc,9,(80, 60))
    try:
        rospy.init_node('thermal_img')
        r = rospy.Rate(rospy.get_param('refresh_rate'))
        img_stream = LeptonThermalImage()
        img_pub = rospy.Publisher('thermal_img',Image,queue_size=queue_size)
        temp_pub = rospy.Publisher('pxl_temps',Tempuratures,queue_size=queue_size)
        compressed_img_pub = rospy.Publisher('compressed_thermal_img',CompressedImage,queue_size=queue_size)
        while not rospy.is_shutdown():
            temp_msg, img_msg, compressed_img_msg = img_stream.get_img()
            temp_pub.publish(temp_msg)
            img_pub.publish(img_msg)
            compressed_img_pub.publish(compressed_img_msg)
            r.sleep()
    except rospy.ROSInterruptException: 
        print('Thermal image node failed!') 
        img_stream.cleanup()
        pass 
