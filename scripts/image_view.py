import cv2 as cv
import rospy as rp
import numpy as np

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from matplotlib import pyplot as plt

import threading


class imageViewNode():
    '''
      A node with a basic GUI to show the spinnaker images
      using OpenCV's high GUI
    '''

    def __init__(self) -> None:
        rp.init_node("image_viewer")

        # Setup Subscribers
        self.l_image_sub = rp.Subscriber(
            "left_camera/image_raw", Image, self.l_image_callback, queue_size=1)
        self.r_image_sub = rp.Subscriber(
            "right_camera/image_raw", Image, self.r_image_callback, queue_size=1)

        # ROS CV bridge
        self.bridge = CvBridge()

        # Playback stuff
        self.l_image_mat = np.ones((512, 512, 1), np.uint8)
        self.r_image_mat = np.ones((512, 512, 1), np.uint8)
        self.gui_mat = np.ones((512, 512, 1), np.uint8)
        self.update_image = True
        self.show_left = True
        self.show_hist = False
        self.center_point = [256, 256]
        self.zoom = 1

        # Setup GUI
        self.sleep_time = 40
        t = threading.Thread(target=self.show_gui)
        t.start()

        rp.spin()

    def l_image_callback(self, data):
        if self.update_image:
            image_mat = self.bridge.imgmsg_to_cv2(
                data, desired_encoding="passthrough")
            self.l_image_mat = cv.rotate(image_mat, cv.ROTATE_180)

    def r_image_callback(self, data):
        if self.update_image:
            image_mat = self.bridge.imgmsg_to_cv2(
                data, desired_encoding="passthrough")
            self.r_image_mat = cv.rotate(image_mat, cv.ROTATE_180)

    def on_mouse_event(self, event, x, y, flags, param):
        if event is cv.EVENT_FLAG_LBUTTON:
            self.center_point = [x, y]

    def trackbar_callback(self, x):
        if x == 0:
            x == 1
        self.zoom = x

    def digital_zoom(self):
        if self.zoom > 1:
            height = self.gui_mat.shape[1]
            width = self.gui_mat.shape[1]
            col_start = int(
                self.center_point[1] - (height / (2 * self.zoom)))
            if col_start < 0:
                col_start = 0

            row_start = int(
                self.center_point[0] - (width / (2 * self.zoom)))
            if row_start < 0:
                row_start = 0

            zoom_w = int(width / self.zoom)
            zoom_h = int(height / self.zoom)
            zoom_roi = self.gui_mat[col_start:col_start +
                                    zoom_w, row_start:row_start+zoom_h]
            self.gui_mat = cv.resize(
                zoom_roi, [width, height], interpolation=cv.INTER_NEAREST)
    
    def get_hist(self):
      hist = cv.calcHist([self.gui_mat], [0], None, [256], [0, 256])
      plt.plot(hist)
      plt.show()

    def show_gui(self):
        main_window = "DIC feed"
        cv.namedWindow(main_window, flags=cv.WINDOW_KEEPRATIO)
        cv.setMouseCallback(main_window, self.on_mouse_event)
        cv.createTrackbar('zoom', main_window, 1, 50, self.trackbar_callback)
        while not rp.is_shutdown():
            # Update image for GUI
            if self.show_left:
                self.gui_mat = self.l_image_mat
            else:
                self.gui_mat = self.r_image_mat

            self.digital_zoom()

            if self.show_hist:
              self.get_hist()

            # Show image
            cv.imshow(main_window, self.gui_mat)
            key = cv.waitKey(self.sleep_time)

            # Update parameters from keyboard events
            if key == ord(' '):
                self.update_image = not self.update_image
            elif key == ord('c'):
                self.show_left = not self.show_left
            elif key == ord('h'):
                self.show_hist = not self.show_hist

        cv.destroyAllWindows()


# Start Node
image_gui = imageViewNode()
