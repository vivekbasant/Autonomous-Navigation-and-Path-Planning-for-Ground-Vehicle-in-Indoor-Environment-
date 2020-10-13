import numpy as np
import cv2
#import pptk
#from open3d import *
import time
import serial
import struct
ser = serial.Serial('COM9', 9600) 
time.sleep(1)  

from numpy import *


global camera_matrix, dist_coeff
with np.load('c1R.npz') as X:
    mtxR,distR,rvecsR,tvecsR = [X[i] for i in ('mtxR','distR','rvecsR','tvecsR')]

global camera_matrix, dist_coeff
with np.load('c1L.npz') as X:
    mtxL,distL,rvecsL,tvecsL = [X[i] for i in ('mtxL','distL','rvecsL','tvecsL')]


global camera_matrix, dist_coeff
with np.load('s1Calibrate.npz') as X:
    retS, MLS, dLS, MRS, dRS, R, T, E, F = [X[i] for i in ('retS', 'MLS', 'dLS', 'MRS', 'dRS', 'R', 'T', 'E', 'F')]




    

global camera_matrix, dist_coeff
with np.load('s1Rectify.npz') as X:
    RL, RR, PL, PR, Q, roiL, roiR = [X[i] for i in ('RL', 'RR', 'PL', 'PR', 'Q', 'roiL', 'roiR')]



print(mtxR,mtxL)

Left_Stereo_Map= cv2.initUndistortRectifyMap(MLS, dLS, RL, PL,
                                             (640,480), cv2.CV_16SC2)   # cv2.CV_16SC2 this format enables us the programme to work faster
Right_Stereo_Map= cv2.initUndistortRectifyMap(MRS, dRS, RR, PR,
                                              (640,480), cv2.CV_16SC2)
window_size = 5
min_disp = 18
num_disp = 130-min_disp
stereo = cv2.StereoSGBM_create(minDisparity = min_disp,
    numDisparities = num_disp,
    blockSize = window_size,
    uniquenessRatio = 10,
    speckleWindowSize = 100,
    speckleRange = 1,
    disp12MaxDiff = 5,
    P1 = 8*3*window_size**2,                           
    P2 = 32*3*window_size**2)

# Used for the filtered image
stereoR=cv2.ximgproc.createRightMatcher(stereo) # Create another stereo for right this time

# WLS FILTER Parameters
lmbda = 80000
sigma = 1.8
visual_multiplier = 1.0
 
wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=stereo)
wls_filter.setLambda(lmbda)
wls_filter.setSigmaColor(sigma)
kernel= np.ones((3,3),np.uint8)



# Copyright 2017 The TensorFlow Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

"""A set of functions that are used for visualization.

These functions often receive an image, perform some visualization on the image.
The functions do not return a value, instead they modify the image itself.

"""
import collections
import numpy as np
import PIL.Image as Image
import PIL.ImageColor as ImageColor
import PIL.ImageDraw as ImageDraw
import PIL.ImageFont as ImageFont
import six
import tensorflow as tf


Dcubic = []

_TITLE_LEFT_MARGIN = 10
_TITLE_TOP_MARGIN = 10
STANDARD_COLORS = [
    'AliceBlue', 'Chartreuse', 'Aqua', 'Aquamarine', 'Azure', 'Beige', 'Bisque',
    'BlanchedAlmond', 'BlueViolet', 'BurlyWood', 'CadetBlue', 'AntiqueWhite',
    'Chocolate', 'Coral', 'CornflowerBlue', 'Cornsilk', 'Crimson', 'Cyan',
    'DarkCyan', 'DarkGoldenRod', 'DarkGrey', 'DarkKhaki', 'DarkOrange',
    'DarkOrchid', 'DarkSalmon', 'DarkSeaGreen', 'DarkTurquoise', 'DarkViolet',
    'DeepPink', 'DeepSkyBlue', 'DodgerBlue', 'FireBrick', 'FloralWhite',
    'ForestGreen', 'Fuchsia', 'Gainsboro', 'GhostWhite', 'Gold', 'GoldenRod',
    'Salmon', 'Tan', 'HoneyDew', 'HotPink', 'IndianRed', 'Ivory', 'Khaki',
    'Lavender', 'LavenderBlush', 'LawnGreen', 'LemonChiffon', 'LightBlue',
    'LightCoral', 'LightCyan', 'LightGoldenRodYellow', 'LightGray', 'LightGrey',
    'LightGreen', 'LightPink', 'LightSalmon', 'LightSeaGreen', 'LightSkyBlue',
    'LightSlateGray', 'LightSlateGrey', 'LightSteelBlue', 'LightYellow', 'Lime',
    'LimeGreen', 'Linen', 'Magenta', 'MediumAquaMarine', 'MediumOrchid',
    'MediumPurple', 'MediumSeaGreen', 'MediumSlateBlue', 'MediumSpringGreen',
    'MediumTurquoise', 'MediumVioletRed', 'MintCream', 'MistyRose', 'Moccasin',
    'NavajoWhite', 'OldLace', 'Olive', 'OliveDrab', 'Orange', 'OrangeRed',
    'Orchid', 'PaleGoldenRod', 'PaleGreen', 'PaleTurquoise', 'PaleVioletRed',
    'PapayaWhip', 'PeachPuff', 'Peru', 'Pink', 'Plum', 'PowderBlue', 'Purple',
    'Red', 'RosyBrown', 'RoyalBlue', 'SaddleBrown', 'Green', 'SandyBrown',
    'SeaGreen', 'SeaShell', 'Sienna', 'Silver', 'SkyBlue', 'SlateBlue',
    'SlateGray', 'SlateGrey', 'Snow', 'SpringGreen', 'SteelBlue', 'GreenYellow',
    'Teal', 'Thistle', 'Tomato', 'Turquoise', 'Violet', 'Wheat', 'White',
    'WhiteSmoke', 'Yellow', 'YellowGreen'
]


def save_image_array_as_png(image, output_path):
  """Saves an image (represented as a numpy array) to PNG.

  Args:
    image: a numpy array with shape [height, width, 3].
    output_path: path to which image should be written.
  """
  image_pil = Image.fromarray(np.uint8(image)).convert('RGB')
  with tf.gfile.Open(output_path, 'w') as fid:
    image_pil.save(fid, 'PNG')


def encode_image_array_as_png_str(image):
  """Encodes a numpy array into a PNG string.

  Args:
    image: a numpy array with shape [height, width, 3].

  Returns:
    PNG encoded image string.
  """
  image_pil = Image.fromarray(np.uint8(image))
  output = six.BytesIO()
  image_pil.save(output, format='PNG')
  png_string = output.getvalue()
  output.close()
  return png_string


def draw_bounding_box_on_image_array(image,
                                     ymin,
                                     xmin,
                                     ymax,
                                     xmax,
                                     color='red',
                                     thickness=4,
                                     display_str_list=(),
                                     use_normalized_coordinates=True):
  """Adds a bounding box to an image (numpy array).

  Args:
    image: a numpy array with shape [height, width, 3].
    ymin: ymin of bounding box in normalized coordinates (same below).
    xmin: xmin of bounding box.
    ymax: ymax of bounding box.
    xmax: xmax of bounding box.
    color: color to draw bounding box. Default is red.
    thickness: line thickness. Default value is 4.
    display_str_list: list of strings to display in box
                      (each to be shown on its own line).
    use_normalized_coordinates: If True (default), treat coordinates
      ymin, xmin, ymax, xmax as relative to the image.  Otherwise treat
      coordinates as absolute.
  """
  image_pil = Image.fromarray(np.uint8(image)).convert('RGB')
  draw_bounding_box_on_image(image_pil, ymin, xmin, ymax, xmax, color,
                             thickness, display_str_list,
                             use_normalized_coordinates)
  np.copyto(image, np.array(image_pil))


def draw_bounding_box_on_image(image,
                               ymin,
                               xmin,
                               ymax,
                               xmax,
                               color='red',
                               thickness=4,
                               display_str_list=(),
                               use_normalized_coordinates=True):
  """Adds a bounding box to an image.

  Each string in display_str_list is displayed on a separate line above the
  bounding box in black text on a rectangle filled with the input 'color'.

  Args:
    image: a PIL.Image object.
    ymin: ymin of bounding box.
    xmin: xmin of bounding box.
    ymax: ymax of bounding box.
    xmax: xmax of bounding box.
    color: color to draw bounding box. Default is red.
    thickness: line thickness. Default value is 4.
    display_str_list: list of strings to display in box
                      (each to be shown on its own line).
    use_normalized_coordinates: If True (default), treat coordinates
      ymin, xmin, ymax, xmax as relative to the image.  Otherwise treat
      coordinates as absolute.
  """
  draw = ImageDraw.Draw(image)
  im_width, im_height = image.size
  if use_normalized_coordinates:
    (left, right, top, bottom) = (xmin * im_width, xmax * im_width,
                                  ymin * im_height, ymax * im_height)
  else:
    (left, right, top, bottom) = (xmin, xmax, ymin, ymax)
  draw.line([(left, top), (left, bottom), (right, bottom),
             (right, top), (left, top)], width=thickness, fill=color)
  #print(left,right,top,bottom)
  try:
    font = ImageFont.truetype('arial.ttf', 24)
  except IOError:
    font = ImageFont.load_default()

  text_bottom = top
  # Reverse list and print from bottom to top.
  for display_str in display_str_list[::-1]:
    text_width, text_height = font.getsize(display_str)
    margin = np.ceil(0.05 * text_height)
    draw.rectangle(
        [(left, text_bottom - text_height - 2 * margin), (left + text_width,
                                                          text_bottom)],
        fill=color)
    draw.text(
        (left + margin, text_bottom - text_height - margin),
        display_str,
        fill='black',
        font=font)
    text_bottom -= text_height - 2 * margin


def draw_bounding_boxes_on_image_array(image,
                                       boxes,
                                       color='red',
                                       thickness=4,
                                       display_str_list_list=()):
  """Draws bounding boxes on image (numpy array).

  Args:
    image: a numpy array object.
    boxes: a 2 dimensional numpy array of [N, 4]: (ymin, xmin, ymax, xmax).
           The coordinates are in normalized format between [0, 1].
    color: color to draw bounding box. Default is red.
    thickness: line thickness. Default value is 4.
    display_str_list_list: list of list of strings.
                           a list of strings for each bounding box.
                           The reason to pass a list of strings for a
                           bounding box is that it might contain
                           multiple labels.

  Raises:
    ValueError: if boxes is not a [N, 4] array
  """
  image_pil = Image.fromarray(image)
  draw_bounding_boxes_on_image(image_pil, boxes, color, thickness,
                               display_str_list_list)
  np.copyto(image, np.array(image_pil))


def draw_bounding_boxes_on_image(image,
                                 boxes,
                                 color='red',
                                 thickness=4,
                                 display_str_list_list=()):
  """Draws bounding boxes on image.

  Args:
    image: a PIL.Image object.
    boxes: a 2 dimensional numpy array of [N, 4]: (ymin, xmin, ymax, xmax).
           The coordinates are in normalized format between [0, 1].
    color: color to draw bounding box. Default is red.
    thickness: line thickness. Default value is 4.
    display_str_list_list: list of list of strings.
                           a list of strings for each bounding box.
                           The reason to pass a list of strings for a
                           bounding box is that it might contain
                           multiple labels.

  Raises:
    ValueError: if boxes is not a [N, 4] array
  """
  boxes_shape = boxes.shape
  if not boxes_shape:
    return
  if len(boxes_shape) != 2 or boxes_shape[1] != 4:
    raise ValueError('Input must be of size [N, 4]')
  for i in range(boxes_shape[0]):
    display_str_list = ()
    if display_str_list_list:
      display_str_list = display_str_list_list[i]
      
    draw_bounding_box_on_image(image, boxes[i, 0], boxes[i, 1], boxes[i, 2],
                               boxes[i, 3], color, thickness, display_str_list)


def draw_keypoints_on_image_array(image,
                                  keypoints,
                                  color='red',
                                  radius=2,
                                  use_normalized_coordinates=True):
  """Draws keypoints on an image (numpy array).

  Args:
    image: a numpy array with shape [height, width, 3].
    keypoints: a numpy array with shape [num_keypoints, 2].
    color: color to draw the keypoints with. Default is red.
    radius: keypoint radius. Default value is 2.
    use_normalized_coordinates: if True (default), treat keypoint values as
      relative to the image.  Otherwise treat them as absolute.
  """
  image_pil = Image.fromarray(np.uint8(image)).convert('RGB')
  draw_keypoints_on_image(image_pil, keypoints, color, radius,
                          use_normalized_coordinates)
  np.copyto(image, np.array(image_pil))


def draw_keypoints_on_image(image,
                            keypoints,
                            color='red',
                            radius=2,
                            use_normalized_coordinates=True):
  """Draws keypoints on an image.

  Args:
    image: a PIL.Image object.
    keypoints: a numpy array with shape [num_keypoints, 2].
    color: color to draw the keypoints with. Default is red.
    radius: keypoint radius. Default value is 2.
    use_normalized_coordinates: if True (default), treat keypoint values as
      relative to the image.  Otherwise treat them as absolute.
  """
  draw = ImageDraw.Draw(image)
  im_width, im_height = image.size
  keypoints_x = [k[1] for k in keypoints]
  keypoints_y = [k[0] for k in keypoints]
  if use_normalized_coordinates:
    keypoints_x = tuple([im_width * x for x in keypoints_x])
    keypoints_y = tuple([im_height * y for y in keypoints_y])
  for keypoint_x, keypoint_y in zip(keypoints_x, keypoints_y):
    draw.ellipse([(keypoint_x - radius, keypoint_y - radius),
                  (keypoint_x + radius, keypoint_y + radius)],
                 outline=color, fill=color)


def draw_mask_on_image_array(image, mask, color='red', alpha=0.7):
  """Draws mask on an image.

  Args:
    image: uint8 numpy array with shape (img_height, img_height, 3)
    mask: a float numpy array of shape (img_height, img_height) with
      values between 0 and 1
    color: color to draw the keypoints with. Default is red.
    alpha: transparency value between 0 and 1. (default: 0.7)

  Raises:
    ValueError: On incorrect data type for image or masks.
  """
  if image.dtype != np.uint8:
    raise ValueError('`image` not of type np.uint8')
  if mask.dtype != np.float32:
    raise ValueError('`mask` not of type np.float32')
  if np.any(np.logical_or(mask > 1.0, mask < 0.0)):
    raise ValueError('`mask` elements should be in [0, 1]')
  rgb = ImageColor.getrgb(color)
  pil_image = Image.fromarray(image)

  solid_color = np.expand_dims(
      np.ones_like(mask), axis=2) * np.reshape(list(rgb), [1, 1, 3])
  pil_solid_color = Image.fromarray(np.uint8(solid_color)).convert('RGBA')
  pil_mask = Image.fromarray(np.uint8(255.0*alpha*mask)).convert('L')
  pil_image = Image.composite(pil_solid_color, pil_image, pil_mask)
  np.copyto(image, np.array(pil_image.convert('RGB')))


def visualize_boxes_and_labels_on_image_array(image,
                                              boxes,
                                              classes,
                                              scores,
                                              category_index,
                                              instance_masks=None,
                                              keypoints=None,
                                              use_normalized_coordinates=False,
                                              max_boxes_to_draw=20,
                                              min_score_thresh=.5,
                                              agnostic_mode=False,
                                              line_thickness=4):
  """Overlay labeled boxes on an image with formatted scores and label names.

  This function groups boxes that correspond to the same location
  and creates a display string for each detection and overlays these
  on the image.  Note that this function modifies the image array in-place
  and does not return anything.

  Args:
    image: uint8 numpy array with shape (img_height, img_width, 3)
    boxes: a numpy array of shape [N, 4]
    classes: a numpy array of shape [N]
    scores: a numpy array of shape [N] or None.  If scores=None, then
      this function assumes that the boxes to be plotted are groundtruth
      boxes and plot all boxes as black with no classes or scores.
    category_index: a dict containing category dictionaries (each holding
      category index `id` and category name `name`) keyed by category indices.
    instance_masks: a numpy array of shape [N, image_height, image_width], can
      be None
    keypoints: a numpy array of shape [N, num_keypoints, 2], can
      be None
    use_normalized_coordinates: whether boxes is to be interpreted as
      normalized coordinates or not.
    max_boxes_to_draw: maximum number of boxes to visualize.  If None, draw
      all boxes.
    min_score_thresh: minimum score threshold for a box to be visualized
    agnostic_mode: boolean (default: False) controlling whether to evaluate in
      class-agnostic mode or not.  This mode will display scores but ignore
      classes.
    line_thickness: integer (default: 4) controlling line width of the boxes.
  """
  # Create a display string (and color) for every box location, group any boxes
  # that correspond to the same location.
  box_to_display_str_map = collections.defaultdict(list)
  box_to_color_map = collections.defaultdict(str)
  box_to_instance_masks_map = {}
  box_to_keypoints_map = collections.defaultdict(list)
  if not max_boxes_to_draw:
    max_boxes_to_draw = boxes.shape[0]
 

  #send = array([[0,0,0,0]])
  send = []
    

  for i in range(min(max_boxes_to_draw, boxes.shape[0])):
    if scores is None or scores[i] > min_score_thresh:
      box = tuple(boxes[i].tolist())
      top=boxes[i,0]
      left=boxes[i,1]
      bottom=boxes[i,2]
      right=boxes[i,3]
      cx= ((right+left)/2)
      cy= (bottom+top)/2
      a = int((640*cx))
      b = int(480*cy)+60
      
      if (a>638):
        a=638
      if (b>478):
        b=478
     
      righto=int(right*640)
      lefto=int(left*640)
      topo=int(top*480)
      bottomo=int(bottom*480)
      width=int(righto-lefto)
      height=int(bottomo-topo)
      
        
      
      if (a==340):
        angle=90
      else:
        angle=math.degrees(math.atan((b-240)/(a-340)))
      #m = array([['Mon',18,20,22,17]
      
      
      
     
    
     
      dist=coords_mouse_disp_Focal(a,b)
      d1=int((((a-320)**2) +((b-240)**2) )**(.5))
           
      #dist=Cubic(a,b)
      reald1=(((dist*d1)/777))                       #realdist=(pixel_dist*dist)/focal_length
      reald1 = np.around(reald1,decimals=2)

      
      widthReal=(((dist*width)/777))/.06
      heightReal=(((dist*height)/777))/.06
      widthReal= np.around(widthReal,decimals=2)
      heightReal = np.around(heightReal,decimals=2)
      
      angle3d=math.degrees(math.atan(reald1/dist))
      angle3d = np.around(angle3d,decimals=2)
      if (a==340):
        angle3d=0    
      if(a<340):
        angle3d = angle3d*(-1)
      xsin = abs(dist*math.sin(angle3d*math.pi/180))/.06
      ycos = abs(dist*math.cos(angle3d*math.pi/180))/.06    
    
      
     # widthReal=int(((dist*width)/5850)*7.3)
      #heightReal=int(((dist*height)/5850)*7.3)
      if(dist<1 and widthReal<5 and heightReal<6):
          send.append([widthReal,heightReal,xsin,ycos,angle3d])
      #print('Distance: '+ str(Distance)+' m')      
      if instance_masks is not None:
        box_to_instance_masks_map[box] = instance_masks[i]
      if keypoints is not None:
        box_to_keypoints_map[box].extend(keypoints[i])
      if scores is None:
        box_to_color_map[box] = 'black'
      else:
        if not agnostic_mode:
          if classes[i] in category_index.keys():
            class_name = category_index[classes[i]]['name']
          else:
            class_name = 'N/A'
          display_str = '{}:{}:{}:{}'.format(
              angle3d,dist,widthReal,heightReal)
           #display_str = '{}'.format(dist)
              #int(100*scores[i]),int((640*cx)),int((480*cy)))
        else:
          display_str = 'score: {}%'.format(int(100 * scores[i]))
        box_to_display_str_map[box].append(display_str)
        if agnostic_mode:
          box_to_color_map[box] = 'DarkOrange'
        else:
          box_to_color_map[box] = STANDARD_COLORS[
              classes[i] % len(STANDARD_COLORS)]

  # Draw all boxes onto image.
  for box, color in box_to_color_map.items():
    ymin, xmin, ymax, xmax = box
    #print(ymin)
    if instance_masks is not None:
      draw_mask_on_image_array(
          image,
          box_to_instance_masks_map[box],
          color=color
      )
    draw_bounding_box_on_image_array(
        image,
        ymin,
        xmin,
        ymax,
        xmax,
        color=color,
        thickness=line_thickness,
        display_str_list=box_to_display_str_map[box],
        use_normalized_coordinates=use_normalized_coordinates)
    if keypoints is not None:
      draw_keypoints_on_image_array(
          image,
          box_to_keypoints_map[box],
          color=color,
          radius=line_thickness / 2,
          use_normalized_coordinates=use_normalized_coordinates)
  return send

import numpy as np
import os
import six.moves.urllib as urllib
import sys
import tarfile
import tensorflow as tf
import zipfile

from collections import defaultdict
from io import StringIO
from matplotlib import pyplot as plt
from PIL import Image


from utils import label_map_util

from utils import visualization_utils as vis_util


# # Model preparation 
# Any model exported using the `export_inference_graph.py` tool can be loaded here simply by changing `PATH_TO_CKPT` to point to a new .pb file.  
# By default we use an "SSD with Mobilenet" model here. See the [detection model zoo](https://github.com/tensorflow/models/blob/master/object_detection/g3doc/detection_model_zoo.md) for a list of other models that can be run out-of-the-box with varying speeds and accuracies.

# What model to download.
#MODEL_NAME = 'ssd_mobilenet_v1_coco_11_06_2017'
MODEL_NAME = 'ssd_mobilenet_v2_coco_2018_03_29'
MODEL_FILE = MODEL_NAME + '.tar.gz'
DOWNLOAD_BASE = 'http://download.tensorflow.org/models/object_detection/'

# Path to frozen detection graph. This is the actual model that is used for the object detection.
PATH_TO_CKPT = MODEL_NAME + '/frozen_inference_graph.pb'

# List of the strings that is used to add correct label for each box.
PATH_TO_LABELS = os.path.join('data', 'mscoco_label_map.pbtxt')

NUM_CLASSES = 90


# ## Download Model

if not os.path.exists(MODEL_NAME + '/frozen_inference_graph.pb'):
	print ('Downloading the model')
	opener = urllib.request.URLopener()
	opener.retrieve(DOWNLOAD_BASE + MODEL_FILE, MODEL_FILE)
	tar_file = tarfile.open(MODEL_FILE)
	for file in tar_file.getmembers():
	  file_name = os.path.basename(file.name)
	  if 'frozen_inference_graph.pb' in file_name:
	    tar_file.extract(file, os.getcwd())
	print ('Download complete')
else:
	print ('Model already exists')

# ## Load a (frozen) Tensorflow model into memory.

detection_graph = tf.Graph()
with detection_graph.as_default():
  od_graph_def = tf.GraphDef()
  with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
    serialized_graph = fid.read()
    od_graph_def.ParseFromString(serialized_graph)
    tf.import_graph_def(od_graph_def, name='')


# ## Loading label map
# Label maps map indices to category names, so that when our convolution network predicts `5`, we know that this corresponds to `airplane`.  Here we use internal utility functions, but anything that returns a dictionary mapping integers to appropriate string labels would be fine

label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)

import matplotlib.pyplot as plt
import time
import matplotlib.animation as animation

def Cubic(x,y):
        #print x,y,disp[y,x],filteredImg[y,x]
      #print(disp[y,x])
        average=0
        for u in range (-1,2):
            for v in range (-1,2):
                average += disp[y+u,x+v]
        average=average/9
     #   print(average)
        r = -593.97*average**(3) + 1506.8*average**(2) - 1373.1*average + 522.06
        r = np.around(r*0.01,decimals=2)
        Distance = -0.1681458 + 0.9469275*r - 0.2551673*r**(2) + 0.03731158*r**(3) 
        Distance = np.around(Distance*100,decimals=0)
        #if cv2.waitKey(1) & 0xFF == ord('a'):
            #Dcubic.append(Distance)
            #print(Dcubic)
       #print(Distance)
      # print('Distance: '+ str(Distance)+' m')
       #print(x,y)
        return Distance
     #  return 3

def coords_mouse_disp_Focal(x,y):
    #if event == cv2.EVENT_LBUTTONDBLCLK:    
        #print x,y,disp[y,x],filteredImg[y,x]
        average=0
        for u in range (-1,2):
            for v in range (-1,2):
                average += disp[y+u,x+v]
        average=average/9
        #Distance= (513.46*7)/(640*average)
        Distance=(8.5/average)
        
        
        Distance= np.around(Distance*0.01,decimals=2)
        Distance=13.146+(229.92*Distance)+(1309.89*(Distance**2))
        Distance= np.around(Distance*0.01,decimals=2)
        return Distance
        #print(x,y)
       #print(average)
       #print('Distance: '+ str(Distance)+' m')
       #print(x,y)


def coords_mouse_disp_Angle(event,x,y,flags,param):
        
        #print x,y,disp[y,x],filteredImg[y,x]
        average=0
        for u in range (-1,2):
            for v in range (-1,2):
                average += disp[y+u,x+v]
        average=average/9
        Distance= (7.3)/(1.1086*average)
        Distance= np.around(Distance*0.01,decimals=2)
        Distance=Distance*6
        
        #print('Distance: '+ str(Distance)+' m')
        #print(x,y)

        
        
def coords_mouse_disp(event,x,y,flags,param):
    if event == cv2.EVENT_LBUTTONDBLCLK:
        #print x,y,disp[y,x],filteredImg[y,x]
        average=0
        for u in range (-1,2):
            for v in range (-1,2):
                average += disp[y+u,x+v]
        average=average/9
        r = -593.97*average**(3) + 1506.8*average**(2) - 1373.1*average + 522.06
        r = np.around(r*0.01,decimals=2)
        Distance = -0.1681458 + 0.9469275*r - 0.2551673*r**(2) + 0.03731158*r**(3) 
        Distance = np.around(Distance*100,decimals=0)
       # if cv2.waitKey(1) & 0xFF == ord('a'):
            #Dcubic.append(Distance)
            #print(Dcubic)
        #print(x,y)



criteria =(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 25, 0.001)
criteria_stereo= (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 25, 0.001)


xdata = []
ydata = []
 
plt.show()
 
axes = plt.gca()
#axes.set_xlim(0, 30)
#axes.set_ylim(0, 30)
line, = axes.plot(xdata, ydata, 'r-')


class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


def astar(maze, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end
    while len(open_list) > 0:

        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] != 0:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:

            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue

            # Add the child to the open list
            open_list.append(child)
a=[]
y=[]
x= []
angl=[]
dis=[]
r=[]
#d = [(120,340,150,380,50,90)] # ymin,xmin,ymax,xmax,depth,ang
i=0
xoff1= 0
xoff2 = 0
                
yoff1 = 0
yoff2 = 0
		
maze = np.zeros((100,50))
u = 6
flag = 0
s = 3
p =0
o=90
start = (0, 25)
end = (99, 25)
path = astar(maze, start, end)


CamR= cv2.VideoCapture(0)   # When 0 then Right Cam and wenn 2 Left Cam
CamL= cv2.VideoCapture(2)


#frameL = cv2.imread('pictureR0.jpg')
#frameR = cv2.imread('pictureL0.jpg')

with detection_graph.as_default():
  with tf.Session(graph=detection_graph) as sess:
   
   while (True):
    # Start Reading Camera images
    
    retR, frameR= CamR.read()
    retL, frameL= CamL.read()
    
   
      # Visualization of the results of a detection.
    
    # Rectify the images on rotation and alignement
    Left_nice= cv2.remap(frameL,Left_Stereo_Map[0],Left_Stereo_Map[1], cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)  # Rectify the image using the kalibration parameters founds during the initialisation
    Right_nice= cv2.remap(frameR,Right_Stereo_Map[0],Right_Stereo_Map[1], cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)

##    # Draw Red lines
##    for line in range(0, int(Right_nice.shape[0]/20)): # Draw the Lines on the images Then numer of line is defines by the image Size/20
##        Left_nice[line*20,:]= (0,0,255)
##        Right_nice[line*20,:]= (0,0,255)
##
##    for line in range(0, int(frameR.shape[0]/20)): # Draw the Lines on the images Then numer of line is defines by the image Size/20
##        frameL[line*20,:]= (0,255,0)
##        frameR[line*20,:]= (0,255,0)    
        
    # Show the Undistorted images
    #cv2.imshow('Both Images', np.hstack([Left_nice, Right_nice]))
    #cv2.imshow('Normal', np.hstack([frameL, frameR]))

    # Convert from color(BGR) to gray
    grayR= cv2.cvtColor(Right_nice,cv2.COLOR_BGR2GRAY)
    grayL= cv2.cvtColor(Left_nice,cv2.COLOR_BGR2GRAY)

    # Compute the 2 images for the Depth_image
    disp= stereo.compute(grayL,grayR)#.astype(np.float32)/ 16
    dispL= disp
    dispR= stereoR.compute(grayR,grayL)
    dispL= np.int16(dispL)
    dispR= np.int16(dispR)
       # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
    
    
    #ply file generation
    
    #points = cv2.reprojectImageTo3D(disp, Q)
    #print(points)
    #v = pptk.viewer(points)
    #colors = cv2.cvtColor(Rigth_nice, cv2.COLOR_BGR2RGB)
    #print(colors)
    #mask = disp > disp.min()

    #out_points = points[mask]
    #v = pptk.viewer(out_points)
    

    #out_colors = colors[mask]

    #out_fn = 'out21.ply'
    #data = plyfile.PlyData.read('out21.ply')['vertex']
    #xyz = np.c_[data['x'], data['y'], data['z']]
    #rgb = np.c_[data['red'], data['green'], data['blue']]
    #n = np.c_[data['nx'], data['ny'], data['nz']]
    #v = pptk.viewer(xyz)
    #v.attributes(rgb / 255., 0.5 * (1 + n))

    #write_ply('out21.ply', out_points, out_colors)

    #print('%s saved' % 'out21.ply')
    
    
    #pcd = read_point_cloud("out21.ply") # Read the point cloud
    #draw_geometries([pcd]) # Visualize the point cloud     
    
    
    
    
    
    
    
    
    # Using the WLS filter
    filteredImg= wls_filter.filter(dispL,grayL,None,dispR)
    filteredImg = cv2.normalize(src=filteredImg, dst=filteredImg, beta=0, alpha=255, norm_type=cv2.NORM_MINMAX);
    filteredImg = np.uint8(filteredImg)
    #cv2.imshow('Disparity Map', filteredImg)
    disp= ((disp.astype(np.float32)/ 16)-min_disp)/num_disp # Calculation allowing us to have 0 for the most distant object able to detect
    image_np_expanded = np.expand_dims(frameL, axis=0)
    image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
      # Each box represents a part of the image where a particular object was detected.
    boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
    
      
      # Each score represent how level of confidence for each of the objects.
      # Score is shown on the result image, together with the class label.
    scores = detection_graph.get_tensor_by_name('detection_scores:0')
    classes = detection_graph.get_tensor_by_name('detection_classes:0')
    num_detections = detection_graph.get_tensor_by_name('num_detections:0')
      # Actual detection.
    (boxes, scores, classes, num_detections) = sess.run(
          [boxes, scores, classes, num_detections],
          feed_dict={image_tensor: image_np_expanded})
    send_array=visualize_boxes_and_labels_on_image_array(
          frameL,
          np.squeeze(boxes),
          np.squeeze(classes).astype(np.int32),
          np.squeeze(scores),
          category_index,
          use_normalized_coordinates=True,
          line_thickness=2)
   # print(send_list)

    
    ##    # Resize the image for faster executions
##    dispR= cv2.resize(disp,None,fx=0.7, fy=0.7, interpolation = cv2.INTER_AREA)

    # Filtering the Results with a closing filter
    closing= cv2.morphologyEx(disp,cv2.MORPH_CLOSE, kernel) # Apply an morphological filter for closing little "black" holes in the picture(Remove noise) 
    #closing1= cv2.morphologyEx(closing,cv2.MORPH_CLOSE, kernel) 
    # Colors map
    dispc= (closing-closing.min())*255
    dispC= dispc.astype(np.uint8)                                   # Convert the type of the matrix from float32 to uint8, this way you can show the results with the function cv2.imshow()
    disp_Color= cv2.applyColorMap(dispC,cv2.COLORMAP_BONE)         # Change the Color of the Picture into an Ocean Color_Map
    filt_Color= cv2.applyColorMap(filteredImg,cv2.COLORMAP_OCEAN)
    
    filt_Color1= cv2.applyColorMap(filteredImg,cv2.COLORMAP_BONE)
    

    # Show the result for the Depth_image
    cv2.imshow('Disparity', dispC)
    #points = cv2.reprojectImageTo3D(disp, Q)
    #print(points)
    #v = pptk.viewer(points)
    #cv2.imshow('Closing',closing)
    #cv2.imshow('Closing1',closing1)
    #cv2.imshow('dispC',dispC)
    #cv2.imshow('Filtered Color Depth',filt_Color1)
    #points = cv2.reprojectImageTo3D(closing, Q)
    #print(points)
    # Mouse click
    
    ##cv2.setMouseCallback("Filtered Color Depth",coords_mouse_disp,filt_Color1)
    
    #cv2.setMouseCallback("Disparity",coords_mouse_disp_Focal,dispC)
    #cv2.setMouseCallback("Filtered Color Depth",coords_mouse_disp_Angle,filt_Color1)
    # End the Programme
    #cv2.imshow('image',frameL)
    #cv2.imshow('frameR',frameR)
    cv2.imshow('frameL',frameL)
    time.sleep(1)
    y = path[1][0]
    x = path[1][1]
    if(flag>5):
        for i in range(len(send_array)):
            xd=abs(send_array[i][2])
            yd=abs(send_array[i][3])
            width = abs(send_array[i][0])
            length = abs(send_array[i][1])
            if(send_array[i][4] > 0):
                xd1 = int(xd-(width/2)+x)
                xd2 = int(xd1+width)
                yd1 = int(yd-(length/2)+y)
                yd2 = int(yd1 + length)
                if(yd2-yoff2>8):
                    for m in range(xd1-s,xd2+s):
                        for n in range(yd1-s,yd2+s):
                            maze[n][m] = 1
                else:
                
                    if((abs(xd1-xoff1)>2 or abs(xd2-xoff2)>2) and (abs(yd1-yoff1)>1 or abs(yd2-yoff2)>1)):
                        for m in range(xd1-s,xd2+s):
                            for n in range(yd1-s,yd2+s):
                                maze[n][m] = 1
            else:
                xd1 = int(x-xd+(width/2))
                xd2 = int(xd1-width)
                yd1 = int(yd+y-(length/2))
                yd2 = int(yd1 + length)
                if(yd2-yoff2>8):
                    for m in range(xd2-s,xd1+s):
                        for n in range(yd1-s,yd2+s):
                            maze[n][m] = 1
                else:
                    if((abs(xd1-xoff1)>2 or abs(xd2-xoff2)>2) and (abs(yd1-yoff1)>1 or abs(yd2-yoff2)>1)):
                        for m in range(xd2-s,xd1+s):
                            for n in range(yd1-s,yd2+s):
                                maze[n][m] = 1
	
    #i=0
    
            xoff1= xd1
            xoff2 = xd2
                
            yoff1 = yd1
            yoff2 = yd2
    
                   

	
    plt.xlim([0, 50])
    plt.ylim([0, 50])
	
    xdata.append(x)
    ydata.append(y)
    line.set_xdata(xdata)
    line.set_ydata(ydata)
    plt.imshow(maze)
    plt.draw()
    plt.pause(1e-17)
    time.sleep(0.01)

    plt.imshow(maze)
    print(flag)
    flag+=1
    if(flag>7):
        path = astar(maze, (y,x), end)
        t = path[1][1]
        r = path[1][0]
        w = t-x
        e = r-y
        if(w==0):
            o=90
        if(w!=0):
            o = int(math.degrees(math.atan(e/w)))
            if(w<0):
                if(o==0):
                    o=180
                else:
                    o = 90+abs(o)
            else:
                o=o
        dista = math.sqrt(w**2+e**2)
		#l = 90-o
        print(dista)
        p=p+1
        u=str(int(dista))
        if len(u) == 1:
            u = '00'+u
        if len(u) == 2:
            u = '0'+u
        g=str(int(o))
        if len(g) == 1:
            g = '00'+g
        if len(g) == 2:
            g = '0'+g
        data = 's'+str(u) + str(g)+'e'
		#if(y==1):
		#	time.sleep(5)
        ser.write(str.encode(data))
        #print(int(ser.readline()))
        time.sleep(0.5)
        if((y,x)==end):
            plt.close()
        

		
    #cv2.imshow('frameR',frameR)
    #cv2.imshow('frameL',frameL)

    
    
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    

# Save excel
##wb.save("data4.xlsx")
#print(Distance)
# Release the Camerasq
CamR.release()
CamL.release()

#k = cv2.waitKey(0)
cv2.destroyAllWindows()


        
        
