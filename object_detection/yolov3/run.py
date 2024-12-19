#!/usr/bin/env python

# Copyright (c) 2019 Aptiv
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
An example of client-side bounding boxes with basic car controls.

Controls:

	W			 : throttle
	S			 : brake
	AD			 : steer
	Space		 : hand-brake

	ESC			 : quit
"""

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================

import subprocess
import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================
import time

start = time.time()
import carla
import weakref
import random
import cv2

#from pydarknet import Detector, Image

try:
    import pygame
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_SPACE
    from pygame.locals import K_a
    from pygame.locals import K_UP
    from pygame.locals import K_DOWN
    from pygame.locals import K_LEFT
    from pygame.locals import K_RIGHT
    from pygame.locals import K_d
    from pygame.locals import K_s
    from pygame.locals import K_w
    from pygame.locals import K_m
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

VIEW_WIDTH = 640
VIEW_HEIGHT = 640
VIEW_FOV = 90

ffmpeg_cmd = ['ffmpeg',
              '-re',
              '-i', '-',
              '-f', 'rawvideo',
              '-r', '25',
              '-video_size', '640x640',
              '-pixel_format', 'rgb24',
              '-c:v', 'libx264', 
              '-preset', 'ultrafast',
              '-bufsize', '64M',
              '-maxrate', '4M',
              '-f', 'rtsp',
              'rtsp://127.0.0.1:8554/stream']

print(ffmpeg_cmd)
ffmpeg_process = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE, shell=False)

ffmpeg_cmd1 = ['ffmpeg',
              '-re',
              '-i', '-',
              '-f', 'rawvideo',
              '-r', '25',
              '-video_size', '640x640',
              '-pixel_format', 'rgb24',
              '-c:v', 'libx264', 
              '-preset', 'ultrafast',
              '-bufsize', '64M',
              '-maxrate', '4M',
              '-f', 'rtsp',
              'rtsp://127.0.0.1:8555/stream']

print(ffmpeg_cmd1)
ffmpeg_process1 = subprocess.Popen(ffmpeg_cmd1, stdin=subprocess.PIPE, shell=False)
# ==============================================================================
# -- BasicSynchronousClient ----------------------------------------------------
# ==============================================================================


class BasicSynchronousClient(object):
    """
    Basic implementation of a synchronous client.
    """

    def __init__(self):
        self.client = None
        self.world = None
        self.camera = None
        self.depth_camera = None
        self.car = None
        self.image = None
        self.depth_image = None
        self.capture = True
        self.depth_capture = True
        self.counter = 0
        self.depth = None
        self.pose = []
        self.log = False

    def camera_blueprint(self):
        """
        Returns camera blueprint.
        """

        camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(VIEW_WIDTH))
        camera_bp.set_attribute('image_size_y', str(VIEW_HEIGHT))
        camera_bp.set_attribute('fov', str(VIEW_FOV))
        return camera_bp

    def depth_camera_blueprint(self):
        """
        Returns camera blueprint.
        """

        depth_camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        depth_camera_bp.set_attribute('image_size_x', str(VIEW_WIDTH))
        depth_camera_bp.set_attribute('image_size_y', str(VIEW_HEIGHT))
        depth_camera_bp.set_attribute('fov', str(VIEW_FOV))
        return depth_camera_bp

    def set_synchronous_mode(self, synchronous_mode):
        """
        Sets synchronous mode.
        """

        settings = self.world.get_settings()
        settings.synchronous_mode = synchronous_mode
        self.world.apply_settings(settings)

    def setup_car(self):
        """
        Spawns actor-vehicle to be controled.
        """

        car_bp = self.world.get_blueprint_library().filter('tt')[0]
        location = random.choice(self.world.get_map().get_spawn_points())
        self.car = self.world.spawn_actor(car_bp, location)

    def setup_camera(self):
        """
        Spawns actor-camera to be used to render view.
        Sets calibration for client-side boxes rendering.
        """

        camera_transform = carla.Transform(carla.Location(x=-2.0, z=2.8), carla.Rotation(pitch=-15))
        self.camera = self.world.spawn_actor(self.camera_blueprint(), camera_transform, attach_to=self.car)
        weak_self = weakref.ref(self)
        self.camera.listen(lambda image: weak_self().set_image(weak_self, image))

        calibration = np.identity(3)
        calibration[0, 2] = VIEW_WIDTH / 2.0
        calibration[1, 2] = VIEW_HEIGHT / 2.0
        calibration[0, 0] = calibration[1, 1] = VIEW_WIDTH / (2.0 * np.tan(VIEW_FOV * np.pi / 360.0))
        self.camera.calibration = calibration

    def setup_depth_camera(self):
        """
        Spawns actor-camera to be used to render view.
        Sets calibration for client-side boxes rendering.
        """

        depth_camera_transform = carla.Transform(carla.Location(x=-2, z=2.3), carla.Rotation(yaw=180))
        self.depth_camera = self.world.spawn_actor(self.depth_camera_blueprint(), depth_camera_transform,
                                                   attach_to=self.car)
        weak_depth_self = weakref.ref(self)
        self.depth_camera.listen(lambda depth_image: weak_depth_self().set_depth_image(weak_depth_self, depth_image))

        calibration = np.identity(3)
        calibration[0, 2] = VIEW_WIDTH / 2.0
        calibration[1, 2] = VIEW_HEIGHT / 2.0
        calibration[0, 0] = calibration[1, 1] = VIEW_WIDTH / (2.0 * np.tan(VIEW_FOV * np.pi / 360.0))
        self.depth_camera.calibration = calibration

    def control(self, car):
        """
        Applies control to main car based on pygame pressed keys.
        Will return True If ESCAPE is hit, otherwise False to end main loop.
        """

        keys = pygame.key.get_pressed()
        if keys[K_ESCAPE]:
            return True

        control = car.get_control()
        control.throttle = 0

        if keys[K_w] or keys[K_UP]:
            control.throttle = 1
            control.reverse = False
        elif keys[K_s] or keys[K_DOWN]:
            control.throttle = 1
            control.reverse = True
        if keys[K_a] or keys[K_LEFT]:
            control.steer = max(-1., min(control.steer - 0.05, 0))
        elif keys[K_d] or keys[K_RIGHT]:
            control.steer = min(1., max(control.steer + 0.05, 0))
        else:
            control.steer = 0
        control.hand_brake = keys[K_SPACE]
        if keys[K_m]:
            if self.log:
                self.log = False
                np.savetxt('log/pose.txt', self.pose)
            else:
                self.log = True
            pass

        car.apply_control(control)
        return False

    @staticmethod
    def set_image(weak_self, img):
        """
        Sets image coming from camera sensor.
        The self.capture flag is a mean of synchronization - once the flag is
        set, next coming image will be stored.
        """

        self = weak_self()
        if self.capture:
            self.image = img
            self.capture = False

    @staticmethod
    def set_depth_image(weak_depth_self, depth_img):
        """
        Sets image coming from camera sensor.
        The self.capture flag is a mean of synchronization - once the flag is
        set, next coming image will be stored.
        """

        self = weak_depth_self()
        if self.depth_capture:
            self.depth_image = depth_img
            self.depth_capture = False

    def render(self):
        if self.image is not None:
            frame = np.frombuffer(self.image.raw_data, dtype=np.uint8)
            frame = np.reshape(frame, (self.image.height, self.image.width, 4))[:, :, :3]
            ret, frame = cv2.imencode('.jpg', frame)
            ffmpeg_process1.stdin.write(frame.tobytes()) 
    
    def depth_render(self):
        if self.depth_image is not None:
            frame = np.frombuffer(self.depth_image.raw_data, dtype=np.uint8)
            frame = np.reshape(frame, (self.depth_image.height, self.depth_image.width, 4))[:, :, :3]
            ret, frame = cv2.imencode('.jpg', frame)
            ffmpeg_process.stdin.write(frame.tobytes())

    def game_loop(self):
        """
        Main program loop.
        """

        try:
            pygame.init()

            self.client = carla.Client('127.0.0.1', 2000)
            self.client.set_timeout(200.0)
            self.world = self.client.get_world()

            self.setup_car()
            self.setup_camera()
            self.setup_depth_camera()
            #self.display = pygame.display.set_mode((VIEW_WIDTH, VIEW_HEIGHT), pygame.HWSURFACE | pygame.DOUBLEBUF)

            pygame_clock = pygame.time.Clock()

            self.set_synchronous_mode(True)
            vehicles = self.world.get_actors().filter('vehicle.*')

            tm = self.client.get_trafficmanager(3000)
            tm_port = tm.get_port()
            self.car.set_autopilot(True, tm_port)
            while True:
                self.world.tick()
                self.capture = True
                self.depth_capture = True
                pygame_clock.tick_busy_loop(25)
                self.render()
                #pygame.display.flip()
                pygame.event.pump()
                self.depth_render()

        # except Exception as e: print(e)
        finally:

            self.set_synchronous_mode(False)
            self.camera.destroy()
            self.depth_camera.destroy()
            self.car.destroy()
            pygame.quit()
            cv2.destroyAllWindows()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    """
    Initializes the client-side bounding box demo.
    """

    try:
        client = BasicSynchronousClient()
        client.game_loop()
    finally:
        print('EXIT')


if __name__ == '__main__':
    main()