from yolov7.utils.datasets import LoadImages as LI, letterbox, np



class LoadImage(LI):
    def __init__(self, im0s, img_size=640, stride=32):
        
        self.img_size = img_size
        self.stride = stride
        self.files = [im0s]
        self.nf = len(im0s)
        self.video_flag = [False] * self.nf
        self.mode = 'image'
    

    def __next__(self):
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




