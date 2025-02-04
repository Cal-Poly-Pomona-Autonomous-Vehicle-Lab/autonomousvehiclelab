########################################################################
#
# Copyright (c) 2022, STEREOLABS.
#
# All rights reserved.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
########################################################################

"""
    Live camera sample showing the camera information and video in real time and allows to control the different
    settings.
"""

import os
import sys
import cv2
import argparse
import pyzed.sl as sl
import numpy as np
import enum

class AppType(enum.Enum):
    LEFT_AND_RIGHT = 1
    LEFT_AND_DEPTH = 2
    LEFT_AND_DEPTH_16 = 3

def main():

    output_dir = opt.output_path_dir
    avi_output_path = opt.output_avi_file 
    output_as_video = True    
    app_type = AppType.LEFT_AND_RIGHT
    if opt.mode == 1 or opt.mode == 3:
        app_type = AppType.LEFT_AND_DEPTH
    if opt.mode == 4:
        app_type = AppType.LEFT_AND_DEPTH_16
    
    # Check if exporting to AVI or SEQUENCE
    if opt.mode !=0 and opt.mode !=1:
        output_as_video = False

    if not output_as_video and not os.path.isdir(output_dir):
        sys.stdout.write("Input directory doesn't exist. Check permissions or create it.\n",
                         output_dir, "\n")
        exit()

    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.SVGA
    init.camera_fps = 30  # The framerate is lowered to avoid any USB3 bandwidth issues

    cam = sl.Camera()
    status = cam.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        print("Camera Open : "+repr(status)+". Exit program.")
        exit()
    

    cam.set_camera_settings(sl.VIDEO_SETTINGS.BRIGHTNESS, 5)
    cam.set_camera_settings(sl.VIDEO_SETTINGS.CONTRAST, 5)
    # cam.set_camera_settings(sl.VIDEO_SETTINGS.HUE, -0)
    cam.set_camera_settings(sl.VIDEO_SETTINGS.SATURATION, 8)
    cam.set_camera_settings(sl.VIDEO_SETTINGS.SHARPNESS, 2)
    cam.set_camera_settings(sl.VIDEO_SETTINGS.GAIN, 15)
    cam.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, 18)
    # cam.set_camera_settings(sl.VIDEO_SETTINGS.WHITEBALANCE_TEMPERATURE, -1)
    runtime = sl.RuntimeParameters()
    
    # Get image size
    image_size = cam.get_camera_information().camera_configuration.resolution
    width = image_size.widthwweeeeeeeewddddddddddddddddddddd
    height = image_size.height
    width_sbs = width * 2
    
    # Prepare side by side image container equivalent to CV_8UC4
    svo_image_sbs_rgba = np.zeros((height, width_sbs, 4), dtype=np.uint8)

    video_writer = None
    if output_as_video:
        # Create video writer with MPEG-4 part 2 codec
        video_writer = cv2.VideoWriter(avi_output_path,
                                       cv2.VideoWriter_fourcc('M', '4', 'S', '2'),
                                       max(cam.get_camera_information().camera_configuration.fps, 25),
                                       (width_sbs, height))
        if not video_writer.isOpened():
            sys.stdout.write("OpenCV video writer cannot be opened. Please check the .avi file path and write "
                             "permissions.\n")
            cam.close()
            exit()


    left_image = sl.Mat()
    right_image = sl.Mat()
    depth_image = sl.Mat()
    win_name = "Camera Control"
    cv2.namedWindow(win_name)
    print_camera_information(cam)
    key = ''
    frame_num = 0
    while key != 113:  # for 'q' key
        err = cam.grab(runtime) 
        if err == sl.ERROR_CODE.SUCCESS: # Check that a new image is successfully acquired
            cam.retrieve_image(left_image, sl.VIEW.LEFT)
            if app_type == AppType.LEFT_AND_RIGHT:
                cam.retrieve_image(right_image, sl.VIEW.RIGHT)
            elif app_type == AppType.LEFT_AND_DEPTH:
                cam.retrieve_image(right_image, sl.VIEW.DEPTH)
            elif app_type == AppType.LEFT_AND_DEPTH_16:
                cam.retrieve_measure(depth_image, sl.MEASURE.DEPTH)

            if output_as_video:
                # Copy the left image to the left side of SBS image
                svo_image_sbs_rgba[0:height, 0:width, :] = left_image.get_data()

                # Copy the right image to the right side of SBS image
                svo_image_sbs_rgba[0:, width:, :] = right_image.get_data()

                # Convert SVO image from RGBA to RGB
                ocv_image_sbs_rgb = cv2.cvtColor(svo_image_sbs_rgba, cv2.COLOR_RGBA2RGB)

                # Write the RGB image in the video
                video_writer.write(ocv_image_sbs_rgb)
                cv2.imshow(win_name, svo_image_sbs_rgba) #Display image
            else:
                # Generate file names
                filename1 = output_dir +"/"+ ("left%s.png" % str(frame_num).zfill(6))
                filename2 = output_dir +"/"+ (("right%s.png" if app_type == AppType.LEFT_AND_RIGHT
                                           else "depth%s.png") % str(frame_num).zfill(6))
                # Save Left images
                cv2.imwrite(str(filename1), left_image.get_data())

                if app_type != AppType.LEFT_AND_DEPTH_16:
                    # Save right images
                    cv2.imwrite(str(filename2), right_image.get_data())
                else:
                    # Save depth images (convert to uint16)
                    cv2.imwrite(str(filename2), depth_image.get_data().astype(np.uint16))
        else:
            print("Error during capture : ", err)
            break
        
        key = cv2.waitKey(5)
        frame_num+=1
    cv2.destroyAllWindows()

    cam.close()

# Display camera information
def print_camera_information(cam):
    cam_info = cam.get_camera_information()
    print("ZED Model                 : {0}".format(cam_info.camera_model))
    print("ZED Serial Number         : {0}".format(cam_info.serial_number))
    print("ZED Camera Firmware       : {0}/{1}".format(cam_info.camera_configuration.firmware_version,cam_info.sensors_configuration.firmware_version))
    print("ZED Camera Resolution     : {0}x{1}".format(round(cam_info.camera_configuration.resolution.width, 2), cam.get_camera_information().camera_configuration.resolution.height))
    print("ZED Camera FPS            : {0}".format(int(cam_info.camera_configuration.fps)))

if __name__ == "__main__":
    parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('--mode', type = int, required=True, help= " Mode 0 is to export LEFT+RIGHT AVI. \n Mode 1 is to export LEFT+DEPTH_VIEW Avi. \n Mode 2 is to export LEFT+RIGHT image sequence. \n Mode 3 is to export LEFT+DEPTH_View image sequence. \n Mode 4 is to export LEFT+DEPTH_16BIT image sequence.")
    parser.add_argument('--output_avi_file', type=str, help='Path to the output .avi file, if mode includes a .avi export', default = '')
    parser.add_argument('--output_path_dir', type = str, help = 'Path to a directory, where .png will be written, if mode includes image sequence export', default = '')
    opt = parser.parse_args()
    if opt.mode > 4 or opt.mode < 0 :
        print("Mode shoud be between 0 and 4 included. \n Mode 0 is to export LEFT+RIGHT AVI. \n Mode 1 is to export LEFT+DEPTH_VIEW Avi. \n Mode 2 is to export LEFT+RIGHT image sequence. \n Mode 3 is to export LEFT+DEPTH_View image sequence. \n Mode 4 is to export LEFT+DEPTH_16BIT image sequence.")
        exit()
    if opt.mode < 2 and len(opt.output_avi_file)==0:
        print("In mode ",opt.mode,", output_avi_file parameter needs to be specified.")
        exit()
    if opt.mode < 2 and not opt.output_avi_file.endswith(".avi"):
        print("--output_avi_file parameter should be a .avi file but is not : ",opt.output_avi_file,"Exit program.")
        exit()
    if opt.mode >=2  and len(opt.output_path_dir)==0 :
        print("In mode ",opt.mode,", output_path_dir parameter needs to be specified.")
        exit()
    if opt.mode >=2 and not os.path.isdir(opt.output_path_dir):
        print("--output_path_dir parameter should be an existing folder but is not : ",opt.output_path_dir,"Exit program.")
        exit()
    main()