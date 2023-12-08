import matplotlib.pyplot as plt
import numpy as np

def saveImg(savename, img):
    s = img.shape
    fig = plt.figure(figsize=(8, int(8*(s[0]/s[1]))))
    plt.imshow(img)
    plt.axis("off")
    fig.tight_layout()
    fig.savefig(savename+".png", bbox_inches = 'tight', pad_inches = 0)
    np.save(savename+".npy", img)
