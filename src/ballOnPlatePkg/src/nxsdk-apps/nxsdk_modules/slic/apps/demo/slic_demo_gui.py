import os
import tkinter as tk
from PIL import Image, ImageTk
from nxsdk_modules.slic.apps.demo.settings import DemoSettings
from nxsdk_modules.slic.apps.demo.settings_gui import SettingsWindow

class DemoWindow:
    def __init__(self, controller):
        self.window = tk.Tk()
        self.controller = controller
        self.window.title("SLIC Demo on Loihi Neuromorphic Chip")

        # Allow the grid to grow incase of empty space
        self.window.grid_rowconfigure(0, weight=1)
        self.window.grid_columnconfigure(0, weight=1)

        # What happens when user closes the window
        self.window.protocol("WM_DELETE_WINDOW", self.close)
        # Create the settings menu
        self._createMenu()
        # Create the main frame and a canvas in the frame to display images
        self._createCenterFrame()
        # Creates the command frame
        self._createCommandFrame()

    def _createMenu(self):
        # create manu
        self.menubar = tk.Menu(self.window)
        self.window.config(menu=self.menubar)
        settingsMenu = tk.Menu(self.menubar, tearoff=0)
        self.menubar.add_cascade(label="Settings", menu=settingsMenu)
        settingsMenu.add_command(
            label="GUI Settings", command=self._settingsMenu)

    def _createCenterFrame(self):
        self.frameCenter = tk.Frame(self.window, bg="white")
        # Binding KeyPress to function self.keyPress
        self.frameCenter.bind("<KeyPress>", self._keyPress)
        self.frameCenter.pack(fill=tk.BOTH, expand=1)
        self.frameCenter.focus_set()
        self.label_text_var = tk.StringVar()
        self.label_text_var.set("Loihi Intel's Neuromorphic Chip")
        self.showLabel = tk.Label(self.frameCenter, bd=2, height=2,
                               textvariable=self.label_text_var, bg="white", font=DemoSettings.LABEL_FONT)
        self.showLabel.pack()

        # Set the background image incase not training anything
        self.bkgrndImage = Image.open(os.path.dirname(
            os.path.realpath(__file__))+"/image/Intel_white.jpg")
        scale_ratio = float(DemoSettings.IMAGE_WIDTH)/self.bkgrndImage.size[0]
        height = int(self.bkgrndImage.size[1]*scale_ratio)
        image_resize = self.bkgrndImage.resize(
            (DemoSettings.IMAGE_WIDTH, height), Image.BILINEAR)

        # Creating a canvas to put the image or cam image
        self.photo = ImageTk.PhotoImage(image_resize)
        self.imgCav = tk.Canvas(self.frameCenter, bg="white",
                             width=DemoSettings.IMAGE_WIDTH, height=height)
        self.imgCavObj = self.imgCav.create_image(
            0, 0, image=self.photo, anchor=tk.NW)
        self.imgCav.pack()
        
    def _createCommandFrame(self):
        self.cmdFrame = tk.Frame(self.window)
        self.cmdFrame.pack(fill=tk.BOTH, expand=1)

        inferButton = tk.Button(self.cmdFrame, text="Inference",
                             command=self._clickInfer)
        inferButton.grid(row=0, column=0)
        trainButton = tk.Button(self.cmdFrame, text="Training",
                             command=self._clickTrain)
        trainButton.grid(row=0, column=1)

        self.trainEntry = tk. Entry(self.cmdFrame, width=20, bg="white")
        self.trainEntry.grid(row=0, column=2)

    def _clickTrain(self):
        self.frameCenter.focus_set()
        object_label = self.trainEntry.get()
        self.trainEntry.delete(0, 'end')
        if object_label == "":
            return
        print("Running training mode")
        print("SLIC is being trained on ", object_label)
        self.controller.train(object_label)

    def _clickInfer(self):
        print("Running inference mode")
        self.frameCenter.focus_set()
        self.controller.infer(labelIdx=0, force=False)

    def _settingsMenu(self):
        settingsWindow = SettingsWindow(self.window, self)
        self.window.wait_window(settingsWindow.window)

    def _keyPress(self, event):
        self.numkeys = [str(i) for i in range(10)]
        if event.char in self.numkeys:
            print("keypress", event.char)
            key = 10 if event.char == "0" else int(event.char)
            self.controller.infer(labelIdx=key, force=False)

    def start(self):
        self.window.mainloop()

    def updateImageScreen(self, image=None):
        if not image:
            image = self.bkgrndImage
        image_width = DemoSettings.IMAGE_WIDTH
        scale_ratio = float(image_width) / image.size[0]
        height = int(image.size[1] * scale_ratio)
        image_resize = image.resize((image_width, height), Image.BILINEAR)
        self.photo = ImageTk.PhotoImage(image_resize)
        self.imgCav.config(width=image_width, height=height)
        self.imgCav.itemconfig(self.imgCavObj, image=self.photo)

    def updateTextLabel(self, text):
        self.label_text_var.set(text)

    def close(self):
        self.window.destroy()