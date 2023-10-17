import pygame
from PIL import Image
from pygame import camera

from nxsdk_modules.slic.apps.demo import imgproc
from nxsdk_modules.slic.apps.demo.settings import DemoSettings


class WebCam:
    def __init__(self, device, width=800, height=600):
        self.device = device
        # Setup Camera
        pygame.init()
        pygame.camera.init()
        self.cam = pygame.camera.Camera(self.device, (width, height))
        self.cam.start()

    @property
    def camera(self):
        return self.cam

    def capture_image(self, repeat=10, x=800, y=600):
        for i in range(repeat):
            img = self.camera.get_image()
        pil_string_image = pygame.image.tostring(img, "RGBA", False)
        orig = Image.frombytes("RGBA", (x, y), pil_string_image)
        crop = imgproc.crop_to_box(orig.convert('L'), tuple(DemoSettings.WEBCAM_BOX))
        down = imgproc.webcam_downsample(crop,
                                         DemoSettings.COM_WINDOW,
                                         DemoSettings.CROP_MODE,
                                         DemoSettings.APERTURE_MARGIN,
                                         DemoSettings.COMPARATOR_OFFSET)
        return orig, crop, down


