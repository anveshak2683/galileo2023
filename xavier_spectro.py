import cv2
import numpy as np
import rospy
import matplotlib.pyplot as plt
from std_msgs.msg import Int8
import matplotlib
from matplotlib.widgets import Slider
import random

class imgg():
    def __init__(self):
        rospy.init_node('spectro_feed', anonymous = True)
        rospy.Subscriber('halo_status', Int8, self.halo_status_callback)
        self.img1 = cv2.imread("/home/nvidia/caesar2020/spectre.jpeg")
        self.Capture = cv2.VideoCapture(0)

    
    def halo_status_callback(self, data):
        if int(data.data) == 1:
            print('called')
            self.Capture = cv2.VideoCapture(0)
            self.minwavelength = 380
            self.maxwavelength = 750
            ret, img2 = self.Capture.read()
            if ret:
                self.Capture.release()
                # img2 = Image.fromarray(img2)
                image = cv2.cvtColor(img2, cv2.COLOR_BGR2RGB)
                shift_first_image_to_right = 10
                self.Plot_Spectrum(self.img1, self.minwavelength, self.maxwavelength, shift_first_image_to_right)
                frame = self.Plot_DifferenceSpectrum(self.img1, img2, self.minwavelength, self.maxwavelength, shift_first_image_to_right)
                image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                cv2.imwrite('img.jpg', image)

    def intensity_from_img(self, imgarr, shape):
        newarr = np.zeros(shape=(shape[0], shape[1]))
        for i in range(shape[0]):
            for j in range(shape[1]):
                newarr[i][j] = (imgarr[i][j][0] + imgarr[i][j][1] + imgarr[i][j][2])

        finarr = np.zeros(shape[1])
        for i in range(shape[1]):
            sum = 0
            for j in range(shape[0]):
                sum += newarr[j][i]
            finarr[i] = sum
        return finarr

    def wavelength_to_rgb(self, wavelength, gamma=0.8):
        wavelength = float(wavelength)
        if wavelength >= 380 and wavelength <= 750:
            A = 1.
        else:
            A=1
        if wavelength >= 380 and wavelength <= 440:
            attenuation = 0.3 + 0.7 * (wavelength - 380) / (440 - 380)
            R = ((-(wavelength - 440) / (440 - 380)) * attenuation) ** gamma
            G = 0.0
            B = (1.0 * attenuation) ** gamma
        elif wavelength >= 440 and wavelength <= 490:
            R = 0.0
            G = ((wavelength - 440) / (490 - 440)) ** gamma
            B = 1.0
        elif wavelength >= 490 and wavelength <= 510:
            R = 0.0
            G = 1.0
            B = (-(wavelength - 510) / (510 - 490)) ** gamma
        elif wavelength >= 510 and wavelength <= 580:
            R = ((wavelength - 510) / (580 - 510)) ** gamma
            G = 1.0
            B = 0.0
        elif wavelength >= 580 and wavelength <= 645:
            R = 1.0
            G = (-(wavelength - 645) / (645 - 580)) ** gamma
            B = 0.0
        elif wavelength >= 645 and wavelength <= 750:
            attenuation = 0.3 + 0.7 * (750 - wavelength) / (750 - 645)
            R = (1.0 * attenuation) ** gamma
            G = 0.0
            B = 0.0
        else:
            R = 0.0
            G = 0.0
            B = 0.0
        return (R,G,B,A)

    def Make_Spectrum(self, wavelengtharr, finarr):
        clim=[self.minwavelength,self.maxwavelength]
        norm = plt.Normalize(*clim)
        wl = np.arange(clim[0],clim[1]+1,2)
        colorlist = list(zip(norm(wl),[self.wavelength_to_rgb(w) for w in wl]))
        spectralmap = matplotlib.colors.LinearSegmentedColormap.from_list("spectrum", colorlist)

        fig, axs = plt.subplots(1, 1, figsize=(8,4), tight_layout=True)

        plt.plot(wavelengtharr, finarr, color='darkred')
        y = np.linspace(0, 6, 100)
        X,Y = np.meshgrid(wavelengtharr, y)
        extent=(np.min(wavelengtharr), np.max(wavelengtharr), np.min(finarr), np.max(finarr))
        plt.imshow(X, clim=clim,  extent=extent, cmap=spectralmap, aspect='auto')
        plt.xlabel('Wavelength (nm)')
        plt.ylabel('Intensity')
        plt.fill_between(wavelengtharr, finarr, max(finarr), color='w')
        # plt.show()
        fig = plt.gcf()
        canvas = fig.canvas
        canvas.draw()
        buf = canvas.buffer_rgba()
        arr = np.asarray(buf)[:, :, :3]
        return arr

    def reject_outliers(self, data):
        m = 1
        u = np.mean(data)
        s = np.std(data)
        filtered = [e for e in data if (u - m * s < e < u + m * s)]
        return filtered

    def Plot_Spectrum(self, img, minwavelength, maxwavelength, shift = 0):
        imgarr = np.asarray(img)
        shape = np.shape(imgarr)
        finarr = np.roll(self.reject_outliers(self.intensity_from_img(imgarr, shape)), shift)
        wavelengtharr = np.linspace(minwavelength, maxwavelength, len(finarr))
        self.Make_Spectrum(wavelengtharr, finarr/max(finarr))

    def Plot_DifferenceSpectrum(self, img1, img2, minwavelength, maxwavelength, shift = 0):
        img1arr = np.asarray(img1)
        img2arr = np.asarray(img2)
        shape1 = np.shape(img1arr)
        shape2 = np.shape(img2arr)
        finarr1 = self.reject_outliers(self.intensity_from_img(img1arr, shape1))
        finarr2 = self.reject_outliers(self.intensity_from_img(img2arr, shape2))
        min_len = min(len(finarr1), len(finarr2))
        finarr1 = np.roll(finarr1[:min_len], shift)
        finarr2 = finarr2[:min_len]
        finarr = (finarr1/max(finarr1)) - (finarr2/max(finarr2))
        wavelengtharr = np.linspace(minwavelength, maxwavelength, len(finarr))
        arr = self.Make_Spectrum(wavelengtharr, finarr)
        return arr


if __name__ == "__main__":
    imgg = imgg()
    rospy.spin()
