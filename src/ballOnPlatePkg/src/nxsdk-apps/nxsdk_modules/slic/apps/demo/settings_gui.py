import tkinter as tk
from nxsdk_modules.slic.apps.demo.settings import DemoSettings


class SettingsWindow:
    """
        Class representing a popup window to set socket ports and to start socket connection
    """

    def __init__(self, mainWindow, parent):
        self.mainWindow = mainWindow
        self.parent = parent
        self.window = tk.Toplevel(mainWindow)
        self.window.title("Settings")
        self.current_image_width = DemoSettings.IMAGE_WIDTH

        self.time_interval = tk.Label(self.window, bd=0, text="time_interval")
        self.time_interval.grid(row=0, column=0)
        self.time_interval_entry = tk.Entry(self.window, width=50, bg="white")
        self.time_interval_entry.insert(tk.END, DemoSettings.TIME_INTERVAl)
        self.time_interval_entry.grid(row=0, column=1)

        self.image_width = tk.Label(self.window, bd=0, text="image_width")
        self.image_width.grid(row=1, column=0)
        self.image_width_entry = tk.Entry(self.window, width=50, bg="white")
        self.image_width_entry.insert(tk.END, DemoSettings.IMAGE_WIDTH)
        self.image_width_entry.grid(row=1, column=1)

        self.label_font = tk.Label(self.window, bd=0, text="label_font")
        self.label_font.grid(row=2, column=0)
        self.label_font_entry = tk.Entry(self.window, width=50, bg="white")
        self.label_font_entry.insert(tk.END, DemoSettings.LABEL_FONT)
        self.label_font_entry.grid(row=2, column=1)

        self.debug = tk.Label(self.window, bd=0, text="debug")
        self.debug.grid(row=3, column=0)
        self.debug_entry = tk.Entry(self.window, width=50, bg="white")
        self.debug_entry.insert(tk.END, DemoSettings.DEBUG)
        self.debug_entry.grid(row=3, column=1)

        self.crop_mode = tk.Label(self.window, bd=0, text="crop_mode")
        self.crop_mode.grid(row=4, column=0)
        self.crop_mode_entry = tk.Entry(self.window, width=50, bg="white")
        self.crop_mode_entry.insert(tk.END, DemoSettings.CROP_MODE)
        self.crop_mode_entry.grid(row=4, column=1)

        self.apperture_margin = tk.Label(self.window, bd=0, text="apperture_margin")
        self.apperture_margin.grid(row=5, column=0)
        self.apperture_margin_entry = tk.Entry(self.window, width=50, bg="white")
        self.apperture_margin_entry.insert(tk.END, DemoSettings.APERTURE_MARGIN)
        self.apperture_margin_entry.grid(row=5, column=1)

        self.com_window = tk.Label(self.window, bd=0, text="com_window")
        self.com_window.grid(row=6, column=0)
        self.com_window_entry = tk.Entry(self.window, width=50, bg="white")
        self.com_window_entry.insert(tk.END, DemoSettings.COM_WINDOW)
        self.com_window_entry.grid(row=6, column=1)

        self.webcam_box = tk.Label(self.window, bd=0, text="webcam_box")
        self.webcam_box.grid(row=7, column=0)
        self.webcam_box_entry = tk.Entry(self.window, width=50, bg="white")
        self.webcam_box_entry.insert(tk.END, DemoSettings.WEBCAM_BOX)
        self.webcam_box_entry.grid(row=7, column=1)

        self.comparator_offset = tk.Label(self.window, bd=0, text="comparator_offset")
        self.comparator_offset.grid(row=8, column=0)
        self.comparator_offset_entry = tk.Entry(self.window, width=50, bg="white")
        self.comparator_offset_entry.insert(tk.END, DemoSettings.COMPARATOR_OFFSET)
        self.comparator_offset_entry.grid(row=8, column=1)

        self.okButton = tk.Button(self.window, text="Apply", default=tk.ACTIVE, command=self.applyNewSettings)
        self.okButton.grid(row=9, column=0)
        self.clearButton = tk.Button(self.window, text="Clear GUI",
                                     default=tk.ACTIVE, command=self.clearwindow)
        self.clearButton.grid(row=9, column=1)

        self.window.geometry("+%d+%d" % (self.mainWindow.winfo_rootx() + 50,
                                         self.mainWindow.winfo_rooty() + 50))

    def clearwindow(self):
        """
        Resets the Screen
        """
        self.parent.updateImageScreen()
        self.parent.updateTextLabel("Loihi Neuromorphic N2Chip")

    def applyNewSettings(self):
        """
        Applies the new settings
        """

        processWebCamBox = lambda x: [int(i) for i in x.strip().split(' ')]

        DemoSettings.TIME_INTERVAl = float(self.time_interval_entry.get())
        DemoSettings.IMAGE_WIDTH = int(self.image_width_entry.get())
        DemoSettings.LABEL_FONT = self.label_font_entry.get()
        DemoSettings.DEBUG = int(self.debug_entry.get())
        DemoSettings.CROP_MODE = int(self.crop_mode_entry.get())
        DemoSettings.APERTURE_MARGIN = int(self.apperture_margin_entry.get())
        DemoSettings.COM_WINDOW = int(self.com_window_entry.get())
        DemoSettings.WEBCAM_BOX = processWebCamBox(self.webcam_box_entry.get())
        DemoSettings.COMPARATOR_OFFSET = int(self.comparator_offset_entry.get())

        if DemoSettings.DEBUG > 0:
            print("Debug Mode on, shrinking window.")
            DemoSettings.IMAGE_WIDTH = 400

        if DemoSettings.IMAGE_WIDTH != self.current_image_width:
            print("Update image screen size width to ",
                  DemoSettings.IMAGE_WIDTH)
            self.current_image_width = DemoSettings.IMAGE_WIDTH
            self.clearwindow()

