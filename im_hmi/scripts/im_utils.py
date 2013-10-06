def metersFromPixels(row, col, ppm, nrows, ncols):
    xx = 1.0*col/ppm
    yy = 1.0*(nrows - row)/ppm
    return (xx, yy)

def pixelsFromMeters(xx, yy, ppm, nrows, ncols):
    row = nrows - int(round(yy*ppm))
    col = int(round(xx*ppm))
    return (row, col)

def greyFromRGB(red, green, blue):
    grey = 0.2989*red + 0.5870*green + 0.1140*blue
    return grey

def rosFromCVbridgeColor(cv_color):
    red = 255 - cv_color[2]
    green = 255 - cv_color[1]
    blue = 255 - cv_color[0]
    return (red, green, blue)
