"""

This demo calculates multiple things for different scenarios.



IF RUNNING ON A PI, BE SURE TO sudo modprobe bcm2835-v4l2



Here are the defined reference frames:



TAG:

                A y

                |

                |

                |tag center

                O---------> x



CAMERA:



                X--------> x

                | frame center

                |

                |

                V y



F1: Flipped (180 deg) tag frame around x axis

F2: Flipped (180 deg) camera frame around x axis



The attitude of a generic frame 2 respect to a frame 1 can obtained by calculating euler(R_21.T)



We are going to obtain the following quantities:

    > from aruco library we obtain tvec and Rct, position of the tag in camera frame and attitude of the tag

    > position of the Camera in Tag axis: -R_ct.T*tvec

    > Transformation of the camera, respect to f1 (the tag flipped frame): R_cf1 = R_ct*R_tf1 = R_cf*R_f

    > Transformation of the tag, respect to f2 (the camera flipped frame): R_tf2 = Rtc*R_cf2 = R_tc*R_f

    > R_tf1 = R_cf2 an symmetric = R_f



"""



import numpy as np

import cv2

import cv2.aruco as aruco

import sys, time, math

from pyzbar.pyzbar import decode  # Added import for QR code detection

class ArucoSingleTracker():

    def __init__(self,

                id_to_find,

                marker_size,

                camera_matrix,

                camera_distortion,

                camera_size=[640,480],

                show_video=False,

                expected_qr_code_data="https://images.app.goo.gl/ajz5fubB7GLMiS5Y7"

                ):

        

        

        self.id_to_find     = id_to_find

        self.marker_size    = marker_size

        self._show_video    = show_video

        self._camera_matrix = camera_matrix

        self._camera_distortion = camera_distortion

        self.expected_qr_code_data = expected_qr_code_data  # Store the expected QR code data

        

        

            

        self._camera_matrix = camera_matrix

        self._camera_distortion = camera_distortion

        

        self.is_detected    = False

        self._kill          = False

        

        #--- 180 deg rotation matrix around the x axis

        self._R_flip      = np.zeros((3,3), dtype=np.float32)

        self._R_flip[0,0] = 1.0

        self._R_flip[1,1] =-1.0

        self._R_flip[2,2] =-1.0



        #--- Define the aruco dictionary

        self._aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)

        self._parameters  = aruco.DetectorParameters()



        #--- Capture the videocamera (this may also be a video or a picture)

        gst_pipeline = "libcamerasrc ! video/x-raw, width=640, height=480, format=BGR ! videoconvert ! appsink"

        self._cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

        #-- Set the camera size as the one it was calibrated with

        #self._cap.set(cv2.CA_PROP_FRAME_WIDTH, camera_size[0])

        #self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_size[1])



        #-- Font for the text in the image

        self.font = cv2.FONT_HERSHEY_PLAIN



        self._t_read      = time.time()

        self._t_detect    = self._t_read

        self.fps_read    = 0.0

        self.fps_detect  = 0.0    

        

    def detect_qr_code(self, timeout=15, verbose=False):

        """

        Captures frames from the camera and detects QR codes.

        Returns True only if the expected QR code is detected.

        

        Args:

            timeout (int): Maximum time to wait for QR code detection in seconds

            verbose (bool): Whether to print detection information

            

        Returns:

            bool: True if expected QR code detected, False otherwise

        """

        if self.expected_qr_code_data is None:

            if verbose:

                print("No expected QR code data provided.")

            return False

        

        print("Starting QR code detection...")

        start_time = time.time()

        qr_detected = False

        while time.time() - start_time < timeout and not self._kill:

            ret, frame = self._cap.read()

            if not ret:

                print("Failed to capture frame from camera.")

                break

            

            # Detect QR codes in the frame

            detected_objects = decode(frame)

            for obj in detected_objects:

                qr_data = obj.data.decode('utf-8')

                if verbose:

                    print(f"Detected QR Code: {qr_data}")

                if qr_data == self.expected_qr_code_data:

                    if verbose:

                        print("Expected QR code detected!")

                    qr_detected = True

                    break

            

            if qr_detected:

                break

        return qr_detected

        

    def _rotationMatrixToEulerAngles(self,R):

        # Calculates rotation matrix to euler angles

        # The result is the same as MATLAB except the order

        # of the euler angles ( x and z are swapped ).

    

        def isRotationMatrix(R):

            Rt = np.transpose(R)

            shouldBeIdentity = np.dot(Rt, R)

            I = np.identity(3, dtype=R.dtype)

            n = np.linalg.norm(I - shouldBeIdentity)

            return n < 1e-6        

        assert (isRotationMatrix(R))



        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])



        singular = sy < 1e-6



        if not singular:

            x = math.atan2(R[2, 1], R[2, 2])

            y = math.atan2(-R[2, 0], sy)

            z = math.atan2(R[1, 0], R[0, 0])

        else:

            x = math.atan2(-R[1, 2], R[1, 1])

            y = math.atan2(-R[2, 0], sy)

            z = 0



        return np.array([x, y, z])



    def _update_fps_read(self):

        t           = time.time()

        self.fps_read    = 1.0/(t - self._t_read)

        self._t_read      = t

        

    def _update_fps_detect(self):

        t           = time.time()

        self.fps_detect  = 1.0/(t - self._t_detect)

        self._t_detect      = t    

    

    def stop(self):

        """

        Stops the tracker and releases the camera.

        """

        self._kill = True

        if self._cap.isOpened():

            self._cap.release()

        cv2.destroyAllWindows()

            

    def track(self, loop=True, verbose=False, show_video=True):

        """

        Tracks the ArUco marker and returns its position and the captured frame.

        Always shows the live video feed.

        """

        self._kill = False

        marker_found = False

        x = y = z = 0

        frame = None



        while not self._kill:

            # Read the camera frame

            ret, frame = self._cap.read()

            if not ret:

                print("Failed to capture frame from camera.")

                break



            self._update_fps_read()



            # Convert to grayscale

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)



            # Detect ArUco markers

            corners, ids, rejected = aruco.detectMarkers(

                image=gray,

                dictionary=self._aruco_dict,

                parameters=self._parameters,

            )



            if ids is not None and self.id_to_find in ids[0]:

                marker_found = True

                self._update_fps_detect()



                # Estimate pose of the marker

                ret = aruco.estimatePoseSingleMarkers(

                    corners, self.marker_size, self._camera_matrix, self._camera_distortion

                )

                rvec, tvec = ret[0][0, 0, :], ret[1][0, 0, :]

                x, y, z = tvec[0], tvec[1], tvec[2]



                # Draw detected marker

                aruco.drawDetectedMarkers(frame, corners)



                if verbose:

                    print(f"Marker X = {x:.1f}  Y = {y:.1f}  Z = {z:.1f}  - fps = {self.fps_detect:.0f}")



            # Display the frame in a live video feed

            cv2.imshow('Live Feed', frame)

            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):  # Press 'q' to quit

                self._cap.release()

                cv2.destroyAllWindows()

                break



            if not loop:

                return marker_found, x, y, z, frame



        return marker_found, x, y, z, frame



if __name__ == "__main__":

    #--- Define Tag

    id_to_find  = 72

    marker_size  = 25 #- [cm]

    #--- Get the camera calibration path

    calib_path  = "function add karna hai"

    camera_matrix   = np.loadtxt(calib_path+'cameraMatrix_raspi.txt', delimiter=',')

    camera_distortion   = np.loadtxt(calib_path+'cameraDistortion_raspi.txt', delimiter=',')                                      

    #--- Define the expected QR code content before using it 

    expected_qr_code= "https://images.app.goo.gl/ajz5fubB7GLMiS5Y7"

    #--- Create tracker with QR code capability

    aruco_tracker = ArucoSingleTracker(

        id_to_find=72, 

        marker_size=10, 

        show_video=False, 

        camera_matrix=camera_matrix, 

        camera_distortion=camera_distortion,

        expected_qr_code_data=expected_qr_code

    )

    aruco_tracker.track(verbose=True)

    aruco_tracker.stop()

