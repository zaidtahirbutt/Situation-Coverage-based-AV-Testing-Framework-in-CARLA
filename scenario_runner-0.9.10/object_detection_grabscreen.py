import os
import pathlib


if "models" in pathlib.Path.cwd().parts:
  while "models" in pathlib.Path.cwd().parts:
    os.chdir('..')
elif not pathlib.Path('models').exists():
  print("git clone the model")
  #!git clone --depth 1 https://github.com/tensorflow/models


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
from IPython.display import display
from grabscreen import grab_screen
import cv2

from object_detection.utils import ops as utils_ops
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

if tf.test.gpu_device_name(): 
    print('Default GPU Device:{}'.format(tf.test.gpu_device_name()))
else:
   print("Please install GPU version of TF")

# If memory growth is enabled for a PhysicalDevice, the runtime initialization will not allocate all memory on the device. Memory growth cannot be configured on a PhysicalDevice with virtual devices configured.
# physical_devices = tf.config.list_physical_devices('GPU')
# try:
# 	tf.config.experimental.set_memory_growth(physical_devices[0], True)
# except:
#   # Invalid device or cannot modify virtual devices once initialized.
# 	print("couldn't enable GPU memory growth")
  #pass

physical_devices = tf.config.experimental.list_physical_devices('GPU')
assert len(physical_devices) > 0, "Not enough GPU hardware devices available"
tf.config.experimental.set_memory_growth(physical_devices[0], True)


# try:
# 	tf.config.gpu.set_per_process_memory_fraction(0.75)
# 	#tf.config.gpu.set_per_process_memory_growth(True)
# except:
#   # Invalid device or cannot modify virtual devices once initialized.
# 	print("couldn't set_per_process_memory_fraction")



def load_model3(model_name):
  base_url = 'http://download.tensorflow.org/models/object_detection/'
  model_file = model_name + '.tar.gz'
  dir_loc = 'D://scenario_runner-0.9.10//models//research//object_detection//models//'

  model_dir = tf.keras.utils.get_file(
    fname= dir_loc + model_name, 
    origin=base_url + model_file,
    untar=True,
    cache_dir=dir_loc)
    

   
  print(model_dir)

  model_dir = pathlib.Path(model_dir)/"saved_model"
  #model_dir = str(pathlib.Path(model_dir))
 
  #model_dir = model_dir + "//" + "saved_model"
 
  print(model_dir)

  model = tf.saved_model.load(str(model_dir))

  return model

# List of the strings that is used to add correct label for each box.
PATH_TO_LABELS = 'models/research/object_detection/data/mscoco_label_map.pbtxt'
category_index = label_map_util.create_category_index_from_labelmap(PATH_TO_LABELS, use_display_name=True)

model_name = 'ssd_mobilenet_v1_coco_2017_11_17'
#detection_model = load_model(model_name)
detection_model = load_model3(model_name)
#print(detection_model.signatures['serving_default'].inputs)
#print(detection_model.signatures['serving_default'].output_dtypes)
#print(detection_model.signatures['serving_default'].output_shapes)

## Main loop3 GrabScreen with vehicle proximity detection 
model = detection_model

while True:
    #for image_path in TEST_IMAGE_PATHS:
    # the array based representation of the image will be used later in order to prepare the
    # result image with boxes and labels on it.
    #image_np = np.array(Image.open(image_path))
    #screen = cv2.resize(grab_screen(region=(0,40,1280,745)), (800,450))
    screen = cv2.resize(grab_screen(region=(0,50, 1920, 1118)), (800,450))
    #screen = cv2.resize(grab_screen(region=(0,40, 2569, 1440)), (800,450))
    image_np = cv2.cvtColor(screen, cv2.COLOR_BGR2RGB)
    # Actual detection.
    image = image_np
    image = np.asarray(image)
    # The input needs to be a tensor, convert it using `tf.convert_to_tensor`.
    input_tensor = tf.convert_to_tensor(image)
    # The model expects a batch of images, so add an axis with `tf.newaxis`.
    input_tensor = input_tensor[tf.newaxis,...]

    # Run inference
    #model_fn = model.signatures['serving_default']

    #input("Press Enter to continue...")
    with tf.device('/cpu:0'):
      model_fn = model.signatures['serving_default']
    	output_dict = model_fn(input_tensor)

    #input("Press Enter to continue...")


    # All outputs are batches tensors.
    # Convert to numpy arrays, and take index [0] to remove the batch dimension.
    # We're only interested in the first num_detections.
    num_detections = int(output_dict.pop('num_detections'))
    output_dict = {key:value[0, :num_detections].numpy() 
                 for key,value in output_dict.items()}
    output_dict['num_detections'] = num_detections

    # detection_classes should be ints.
    output_dict['detection_classes'] = output_dict['detection_classes'].astype(np.int64)

    # Handle models with masks:
    if 'detection_masks' in output_dict:
        # Reframe the the bbox mask to the image size.
        detection_masks_reframed = utils_ops.reframe_box_masks_to_image_masks(
                  output_dict['detection_masks'], output_dict['detection_boxes'],
                   image.shape[0], image.shape[1])      
        detection_masks_reframed = tf.cast(detection_masks_reframed > 0.5,
                                           tf.uint8)
        output_dict['detection_masks_reframed'] = detection_masks_reframed.numpy()


    #output_dict = run_inference_for_single_image(model, image_np)
    # Visualization of the results of a detection.
    vis_util.visualize_boxes_and_labels_on_image_array(
      image_np,
      output_dict['detection_boxes'],
      output_dict['detection_classes'],
      output_dict['detection_scores'],
      category_index,
      instance_masks=output_dict.get('detection_masks_reframed', None),
      use_normalized_coordinates=True,
      line_thickness=8)
    
    boxes = output_dict['detection_boxes']
    classes = output_dict['detection_classes']
    scores = output_dict['detection_scores']

    for i,b in enumerate(boxes):
            #   car                    bus                  truck
        if classes[i] == 3 or classes[i] == 6 or classes[i] == 8:
            if scores[i] >= 0.5:
                mid_x = (boxes[i][1]+boxes[i][3])/2
                mid_y = (boxes[i][0]+boxes[i][2])/2
                apx_distance = round(((1 - (boxes[i][3] - boxes[i][1]))**4),1)

                #cv2.putText(image_np, '{}'.format(apx_distance), (int(mid_x*image_np.shape[1]),int(mid_y*image_np.shape[0])), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
                cv2.putText(image_np, '{}'.format(apx_distance), (int(mid_x*800),int(mid_y*450)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

                #if apx_distance <=0.5:  # Almost 1 meter
                #if apx_distance <=0.6: # Almost 2.5 meters  
                if apx_distance <=0.7:
                    if mid_x > 0.3 and mid_x < 0.7:  # if it is in the middle!
                    #if mid_x > 0.2 and mid_x < 0.8:  # not so middle
                        #pass
                        #cv2.putText(image_np, 'WARNING!!!', (int(mid_x*800) - 50, int(mid_y*450) - 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,255), 3)
                        cv2.putText(image_np, 'COLLISION-HAZARD!!!', (int(mid_x*800) - 50, int(mid_y*450) - 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,0,255), 3)

    #cv2.imshow('window',cv2.resize(image_np,(800,450)))
    #if cv2.waitKey(25) & 0xFF == ord('q'):
      #cv2.destroyAllWindows()
      #break
        
    #display(Image.fromarray(image_np))
    #cv2.imshow('object Detected', image_np)
    #cv2.imshow('object detection', cv2.resize(image_np, (800,600)))
    cv2.imshow('object detection', cv2.resize(image_np, (400,300)))
    if cv2.waitKey(1) & 0xFF == ord('q'):
    #if cv2.waitKey(25) & 0xFF == ord('q'):
        cv2.destroyAllWindows()
        break