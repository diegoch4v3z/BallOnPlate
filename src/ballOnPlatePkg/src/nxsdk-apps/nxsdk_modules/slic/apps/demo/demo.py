import argparse
import os
import time
from datetime import datetime
from threading import Thread
from PIL import Image
from nxsdk_modules.slic.apps.demo.model import SlicModel
from nxsdk_modules.slic.apps.demo.camera import WebCam
from nxsdk_modules.slic.apps.demo.settings import DemoSettings
from nxsdk_modules.slic.apps.demo.slic_demo_gui import DemoWindow

class Controller:
    """
    Is the Controller Element of the MVC model.
    All the command from the gui ends up here.
    It has instance of model and view, takes
    command from the view, runs the command on model
    and displays the result.
    """
    def __init__(self, videoDev):
        self.model = SlicModel()
        self.view = DemoWindow(self)
        self.camera = WebCam(videoDev)

    def start(self):
        """
        Starts up the main GUI.
        """
        self.view.start()

    def train(self, object_label):
        """
        Spins a thread to do training
        :param object_label: Label of the newly added object
        """
        Thread(target=self._train, args=(
            object_label,)).start()

    def infer(self, labelIdx, force=False):
        """
        Spins a thread to run inference
        """
        Thread(target=self._infer, args=(
            labelIdx, force)).start()

    def capture_and_show_images(self, n=36, interval=10.0 / 36, repeat=2):
        """
        Captures the image from the webcam and displays on the gui
        :param n: No. of images to be taken
        :param interval: Interval at which images will be taken
        :param repeat: No of repetitions of capturing image
        """
        images = []
        for i in range(n):
            orig_img, crop_img, proc_img = self.camera.capture_image(repeat=repeat)
            if DemoSettings.DEBUG == 1:
                img = crop_img
            elif DemoSettings.DEBUG == 2:
                img = proc_img
            else:
                img = orig_img
            images.append(proc_img)
            self.view.updateImageScreen(img)
            time.sleep(interval)
        return images

    def _train(self, objectLabel):
        """
        Captures the image of the new object set from webcam, add it to
        model object set and train the network on it.
        :param objectLabel: Object name associated with new webcam
        """
        self.view.updateTextLabel("Taking pictures of: {0}".format(objectLabel))
        self.model.addObject(objectLabel, self.capture_and_show_images())
        self.view.updateTextLabel("Loihi is training...")
        self.view.updateImageScreen(Image.open(os.path.dirname(
            os.path.realpath(__file__)) + "/image/Intel_white.jpg"))
        start_time = datetime.now()
        self.model.loihi_train()
        end_time = datetime.now()
        train_time = end_time - start_time
        self.view.updateTextLabel("Loihi trained {0} in {1:.2f} seconds! ".format(
                objectLabel, train_time.total_seconds()))

    def _infer(self, labelIdx, force):
        """
        Captures the image of the object to be inferred set from webcam,
        and run inference on it. Make conclusion and display the result
        on the gui.
        :param objectLabel: Object name associated with new webcam
        """
        print("Getting inference image")
        infer_image = self.capture_and_show_images(n = 1, interval=0, repeat=1)[0]
        self.view.updateTextLabel("Asking Loihi what this is...")

        # Send image to loihi and wait for feedback
        is_unsure, is_unknown, got, actual = self.model.loihi_infer(
            infer_image)
        print("Loihi Guess:", (is_unsure, is_unknown, got, actual))

        guessed_is_what_we_trained = got < 20 - self.model.trained
        actual_is_what_we_trained = actual < 20 - self.model.trained

        guessed_name = self.model.get_object_name(got)
        actual_name = self.model.get_object_name(actual)

        print("Loihi output is : ",guessed_name,actual_name)

        if force:
            idx = (20 - self.model.trained + labelIdx - 1) % 20
            print(self.model.trained, labelIdx, idx)
            name = self.model.get_object_name(idx)
            self.view.updateTextLabel("Loihi thinks this is: {0}".format(name))
        elif is_unknown or guessed_is_what_we_trained:
            self.view.updateTextLabel("Loihi thinks this is an Unknown Object!")
        elif is_unsure and not guessed_is_what_we_trained and not actual_is_what_we_trained:
            self.view.updateTextLabel("Loihi thinks this is: {0} but maybe: {1}".format(
                guessed_name, actual_name))
        elif is_unsure and not guessed_is_what_we_trained and actual_is_what_we_trained:
            self.view.updateTextLabel("Loihi thinks this is maybe...: {0} ".format(
                guessed_name))
        else:
            self.view.updateTextLabel("Loihi thinks this is: {0}".format(guessed_name))

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Demo demonstrate SLIC Network')
    parser.add_argument(
    '--video',
    type = str,
    required=True,
    help = 'Path of Capture Device :  --video=</dev/videoN>')
    args = parser.parse_args()

    app = Controller(args.video)
    app.start()





