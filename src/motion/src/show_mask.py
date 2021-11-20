import numpy as np
import cv2

def show_mask_and_annotated_objects(image_, title, mask_ = None, annotations = None,
                                    annotation_type = "bboxes"):
    if (annotations is not None):
        for ann in annotations:
            if (annotation_type == "bboxes"):
                cv2.rectangle(image_, ann[0], ann[1], (100, 200, 200), 3)
            
            elif (annotation_type == "lines"):
                cv2.line(image_, (ann[0], ann[1]), (ann[2], ann[3]), (200, 100, 30), 3)
            
    #image = np.array(image_, np.uint8)

    #if (mask_ is not None):
    #    mask = np.array(mask_, np.float64)
        
    #    if (len(mask.shape) == 2):
    #        mask_3_ch = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
        
    #    else:
    #        mask_3_ch = mask

    #    result = np.concatenate((image, mask_3_ch), axis=0)
    
    #else:
    #    result = image
    
    image = np.array(image_, np.uint8)
    result = np.concatenate((image, mask_), axis=0)

    cv2.imshow(title, result)