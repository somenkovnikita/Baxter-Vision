import cv2
import numpy

# D.D.M. 2016(C)
# Extension for camera

red = (0, 0, 255)


# Search on frame
def find_image(frame, templates, files):
    i = 0
    fm = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    for image in templates:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        x, y, width, height = find_template(fm, image)
        if x > 0 and y > 0 and width > 0 and height > 0:
            # Draw rectangle on image
            cv2.rectangle(frame, (x, y), (x + width, y + height), red)
            print "Found: "
            print files[i]
        i += 1

    return frame


def find_abc(frame, templates, files, bound_low, bound_up):
    i = 0
    frame = find_edges(frame, bound_low, bound_up)
    for image in templates:
        image = find_edges(image, bound_low, bound_up)
        x, y, width, height = find_template(frame, image)
        if x > 0 and y > 0 and width > 0 and height > 0:
            # Draw rectangle on image
            cv2.rectangle(frame, (x, y), (x + width, y + height), red)
            print 'Found: '
            print files[i]
        i += 1

    return frame


# Find image borders
def find_edges(frame, bound_low, bound_up):
    fm = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(fm, bound_low, bound_up, apertureSize=3)
    return apply_morph_filter(edges)


# Morphology filter
def apply_morph_filter(gray_frame):
    kernel = numpy.ones((3, 5))
    return cv2.morphologyEx(gray_frame, cv2.MORPH_GRADIENT, kernel)


# Search image on frame
def find_template(src_image, tmp_image):
    width, height = tmp_image.shape[::]
    res = cv2.matchTemplate(src_image, tmp_image, cv2.TM_CCOEFF_NORMED)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
    position = max_loc

    # IMHO 0.65-0.75 best values
    if max_val > 0.70:
        # x, y, width, height
        return position[0], position[1], width, height

    return 0, 0, 0, 0
