def metersFromPixels(row, col, ppm, nrows, ncols):
    xx = 1.0*col/ppm
    yy = 1.0*(nrows - row)/ppm
    return (xx, yy)

def pixelsFromMeters(xx, yy, ppm, nrows, ncols):
    row = nrows - int(round(yy*ppm))
    col = int(round(xx*ppm))
    return (row, col)

