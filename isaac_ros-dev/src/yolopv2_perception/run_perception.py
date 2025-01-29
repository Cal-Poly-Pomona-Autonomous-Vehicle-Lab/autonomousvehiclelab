import argparse
import time
from pathlib import Path
import cv2
import torch
import os
import sys
import cv2
import argparse
import pyzed.sl as sl
import numpy as np
import enum

# Conclude setting / general reprocessing / plots / metrices / datasets
from utils.utils import \
    time_synchronized,select_device, increment_path,\
    scale_coords,xyxy2xywh,non_max_suppression,split_for_trace_model,\
    driving_area_mask,lane_line_mask,plot_one_box,show_seg_result,letterbox,\
    AverageMeter,\
    LoadImages

def make_parser():
        parser = argparse.ArgumentParser()
        parser.add_argument('--weights', nargs='+', type=str, default='data/weights/yolopv2.pt', help='model.pt path(s)')
        parser.add_argument('--source', type=str, default='data/example.jpg', help='source')  # file/folder, 0 for webcam
        parser.add_argument('--img-size', type=int, default=640, help='inference size (pixels)')
        parser.add_argument('--conf-thres', type=float, default=0.3, help='object confidence threshold')
        parser.add_argument('--iou-thres', type=float, default=0.45, help='IOU threshold for NMS')
        parser.add_argument('--device', default='0', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
        parser.add_argument('--save-conf', action='store_true', help='save confidences in --save-txt labels')
        parser.add_argument('--save-txt', action='store_true', help='save results to *.txt')
        parser.add_argument('--nosave', action='store_true', help='do not save images/videos')
        parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --class 0, or --class 0 2 3')
        parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
        parser.add_argument('--project', default='runs/detect', help='save results to project/name')
        parser.add_argument('--name', default='exp', help='save results to project/name')
        parser.add_argument('--exist-ok', action='store_true', help='existing project/name ok, do not increment')
        return parser

class Percept(object):
        def __init__(self, opt):
            self.model =  torch.jit.load(opt.weights)
            self.source = opt.source
            self.imgsz = opt.img_size
            self.device = select_device(opt.device)
            self.half = self.device.type != 'cpu'  # half precision only supported on CUDA
            self.model = self.model.to(self.device)
            if self.half:
                self.model.half()  # to FP16  
            self.model.eval()

        def detect(self, img):
            inf_time = AverageMeter()
            waste_time = AverageMeter()
            nms_time = AverageMeter()

            # Load model
            stride =32
    
            # Set Dataloader
            vid_path, vid_writer = None, None

            # Run inference
            if self.device.type != 'cpu':
                print("Here!")
                self.model(torch.zeros(1, 3, self.imgsz, self.imgsz).to(self.device).type_as(next(self.model.parameters())))  # run once
            t0 = time.time()
            im0 = cv2.resize(img, (1280,720), interpolation=cv2.INTER_LINEAR)
            img = letterbox(im0, self.imgsz, stride=stride)[0]
            img = img.transpose(2, 0, 1) 
            img = np.ascontiguousarray(img)
            print(f"Image shape after letterbox: {img.shape}, {im0.shape}")
            img = torch.from_numpy(img).to(self.device)
            img = img.half() if self.half else img.float()  # uint8 to fp16/32
            img /= 255.0  # 0 - 255 to 0.0 - 1.0

            if img.ndimension() == 3:
                img = img.unsqueeze(0)

            # Inference
            t1 = time_synchronized()
            [pred,anchor_grid],seg,ll= self.model(img)
            t2 = time_synchronized()

            # waste time: the incompatibility of  torch.jit.trace causes extra time consumption in demo version 
            # but this problem will not appear in offical version 
            tw1 = time_synchronized()
            pred = split_for_trace_model(pred,anchor_grid)
            tw2 = time_synchronized()

            # Apply NMS
            t3 = time_synchronized()
            pred = non_max_suppression(pred, opt.conf_thres, opt.iou_thres, classes=opt.classes, agnostic=opt.agnostic_nms)
            t4 = time_synchronized()
            len(pred)
            da_seg_mask = driving_area_mask(seg)
            ll_seg_mask = lane_line_mask(ll)

            # Process detections
            for i, det in enumerate(pred):  # detections per image 
                # s += '%gx%g ' % img.shape[2:]  # print string
                gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
                if len(det):
                    # Rescale boxes from img_size to im0 size
                    det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                    # Print results
                    for c in det[:, -1].unique():
                        n = (det[:, -1] == c).sum()  # detections per class
                        #s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string
                    
                    # if save_txt:  # Write to file
                    #     xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                    #     line = (cls, *xywh, conf) if opt.save_conf else (cls, *xywh)  # label format
                    #     with open(txt_path + '.txt', 'a') as f:
                    #         f.write(('%g ' * len(line)).rstrip() % line + '\n')

                    # Write results
                    for *xyxy, conf, cls in reversed(det):
                       plot_one_box(xyxy, im0, line_thickness=3)

                # Print time (inference)
                print(f'Done. ({t2 - t1:.3f}s)')
                show_seg_result(im0, (da_seg_mask,ll_seg_mask), is_demo=True)
                return im0

            inf_time.update(t2-t1,img.size(0))
            nms_time.update(t4-t3,img.size(0))
            waste_time.update(tw2-tw1,img.size(0))
            print('inf : (%.4fs/frame)   nms : (%.4fs/frame)' % (inf_time.avg,nms_time.avg))
            print(f'Done. ({time.time() - t0:.3f}s)')

class ZedCam(object):

    def __init__(self):
        init = sl.InitParameters()
        init.camera_resolution = sl.RESOLUTION.SVGA
        init.camera_fps = 30  # The framerate is lowered to avoid any USB3 bandwidth issues

        self.cam = sl.Camera()
        status = self.cam.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            print("Camera Open : "+repr(status)+". Exit program.")
            exit()
        self.cam.set_camera_settings(sl.VIDEO_SETTINGS.BRIGHTNESS, 5)
        self.cam.set_camera_settings(sl.VIDEO_SETTINGS.CONTRAST, 5)
        # self.cam.set_camera_settings(sl.VIDEO_SETTINGS.HUE, -0)
        self.cam.set_camera_settings(sl.VIDEO_SETTINGS.SATURATION, 8)
        self.cam.set_camera_settings(sl.VIDEO_SETTINGS.SHARPNESS, 2)
        self.cam.set_camera_settings(sl.VIDEO_SETTINGS.GAIN, 15)
        self.cam.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, 18)
        # self.cam.set_camera_settings(sl.VIDEO_SETTINGS.WHITEBALANCE_TEMPERATURE, -1)
        self.runtime = sl.RuntimeParameters()
        
        # Get image size
        self.image_size = self.cam.get_camera_information().camera_configuration.resolution
        self.width = self.image_size.width
        self.height = self.image_size.height
    
        # Prepare side by side image container equivalent to CV_8UC4
        self.image_rgba = np.zeros((self.height, self.width, 4), dtype=np.uint8)
    
    def get_image(self):
        left_image = sl.Mat()
        err = self.cam.grab(self.runtime) 
        if err == sl.ERROR_CODE.SUCCESS: # Check that a new image is successfully acquired
            self.cam.retrieve_image(left_image, sl.VIEW.LEFT)
            self.image_rgba[0:self.height, 0:self.width, :] = left_image.get_data()
            image_rgb = cv2.cvtColor(self.image_rgba, cv2.COLOR_RGBA2RGB)
        return image_rgb
    
    def close_cam(self):
        self.cam.close()

    # Display camera information
    def print_camera_information(self):
        cam_info = self.cam.get_camera_information()
        print("ZED Model                 : {0}".format(cam_info.camera_model))
        print("ZED Serial Number         : {0}".format(cam_info.serial_number))
        print("ZED Camera Firmware       : {0}/{1}".format(cam_info.camera_configuration.firmware_version,cam_info.sensors_configuration.firmware_version))
        print("ZED Camera Resolution     : {0}x{1}".format(round(cam_info.camera_configuration.resolution.width, 2), cam.get_camera_information().camera_configuration.resolution.height))
        print("ZED Camera FPS            : {0}".format(int(cam_info.camera_configuration.fps)))

if __name__ == '__main__':
    opt =  make_parser().parse_args()
    print(opt)
    percept = Percept(opt)
    cam = ZedCam()
    # cam.print_camera_information()
    while True:
        img = cam.get_image()
        with torch.no_grad():
            res = percept.detect(img)
            cv2.imshow("Result",res)
        cv2.waitKey(5)

