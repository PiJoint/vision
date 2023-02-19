#!/usr/bin/env python
"""!

    This file is part of PiJoint.

    @package pijoint_vision ai
    @author: Alessandro Mizzaro
    @version: 1.0.0

    CNN utils
"""
from yolov7.utils.datasets import LoadImages as LI, letterbox, np



class LoadImage(LI):
    """!
        Extends LoadImages class
        Source loader for single image
    """
    def __init__(self, im0s, img_size=640, stride=32):
        """!
        @param im0s: image path
        @param img_size: image size
        @param stride: stride
        """
        ## (320, 192) or (416, 256) or (608, 352) for (height, width) 
        self.img_size = img_size 
        
        ## Initialize string
        self.stride = stride
        ## Initialize counter
        self.files = [im0s]
        ## Initialize counter
        self.nf = len(im0s)
        ## Internal flag
        self.video_flag = [False] * self.nf
        ## Internal flag
        self.mode = 'image'
    

    def __next__(self):
        """!
        Get next image
        @return: image

        """
        if self.count == self.nf:
            raise StopIteration
        img0 = self.files[self.count]

       
            # Read image
        self.count += 1
        
        assert img0 is not None, 'Image Not Found '
            #print(f'image {self.count}/{self.nf} {path}: ', end='')

        # Padded resize
        img = letterbox(img0, self.img_size, stride=self.stride)[0]

        # Convert
        img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
        img = np.ascontiguousarray(img)
        return 'tmp', img, img0, None




