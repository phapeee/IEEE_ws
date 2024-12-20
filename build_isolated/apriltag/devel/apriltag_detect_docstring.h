// SPDX-FileCopyrightText: Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
// SPDX-License-Identifier: BSD-3-Clause
const char apriltag_detect_docstring[] =
"AprilTag detector\n"
"\n"
"SYNOPSIS\n"
"\n"
"    import cv2\n"
"    import numpy as np\n"
"    from apriltag import apriltag\n"
"\n"
"    imagepath = '/tmp/tst.jpg'\n"
"    image     = cv2.imread(imagepath, cv2.IMREAD_GRAYSCALE)\n"
"    detector = apriltag(\"tag36h11\")\n"
"\n"
"    detections = detector.detect(image)\n"
"\n"
"    print(\"Saw tags {} at\\n{}\". \\\n"
"          format([d['id']     for d in detections],\n"
"                 np.array([d['center'] for d in detections])))\n"
"\n"
"    ----> Saw tags [3, 5, 7, 8, 10, 10, 14] at\n"
"          [[582.42911184 172.90587335]\n"
"           [703.32149701 271.50587376]\n"
"           [288.1462089  227.01502779]\n"
"           [463.63679264 227.91185418]\n"
"           [ 93.88534443 241.61109765]\n"
"           [121.94062798 237.97010936]\n"
"           [356.46940849 260.20169159]]\n"
"\n"
"DESCRIPTION\n"
"\n"
"The AprilTags visual fiducial system project page is here:\n"
"https://april.eecs.umich.edu/software/apriltag\n"
"\n"
"This is a Python class to provide AprilTags functionality in Python programs. To\n"
"run the detector you\n"
"\n"
"1. Construct an object of type apriltag.apriltag()\n"
"\n"
"2. Invoke the detect() method on this object\n"
"\n"
"The detect() method takes a single argument: an image array. The return value is\n"
"a tuple containing the detections. Each detection is a dict with keys:\n"
"\n"
"- id: integer identifying each detected tag\n"
"\n"
"- center: pixel coordinates of the center of each detection.  NOTE: Please be\n"
"  cautious regarding the image coordinate convention. Here, we define (0,0) as\n"
"  the left-top corner (not the center point) of the left-top-most pixel.\n"
"\n"
"- lb-rb-rt-lt: pixel coordinates of the 4 corners of each detection. The order\n"
"  is left-bottom, right-bottom, right-top, left-top\n"
"\n"
"- hamming: How many error bits were corrected? Note: accepting large numbers of\n"
"  corrected errors leads to greatly increased false positive rates. NOTE: As of\n"
"  this implementation, the detector cannot detect tags with a hamming distance\n"
"  greater than 2.\n"
"\n"
"- margin: A measure of the quality of the binary decoding process: the average\n"
"  difference between the intensity of a data bit versus the decision threshold.\n"
"  Higher numbers roughly indicate better decodes. This is a reasonable measure\n"
"  of detection accuracy only for very small tags-- not effective for larger tags\n"
"  (where we could have sampled anywhere within a bit cell and still gotten a\n"
"  good detection.)\n"
"";
