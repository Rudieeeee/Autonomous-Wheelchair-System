"""Orlosky Pupil Detector — *live camera edition*

This is a refactor of the original **OrloskyPupilDetector.py** to let you pick a
web cam or a video file at launch (the same little Tk GUI that the 3 D tracker
uses).  All the pupil finding logic is unchanged; we've just:

* added a camera selector / file browser GUI
* broken the video loop out into a `process_camera()` routine
* kept `process_video()` for offline files

Run it with `python OrloskyPupilDetectorLive.py`, choose *Start Camera*, and hit
`Space` to pause, `D` for debug overlays, `Q` to quit.
"""
import sys
import cv2
import numpy as np
import math
import os
import matplotlib.pyplot as plt  # only used when debug‑mode is on
import time
import threading
import random
import socket

TEST_MODE = "--test" in sys.argv


last_sent_x = None  # To avoid re-sending the same value
# GUI dimensions and font (copied from 2.2 GUI.py)
WIDTH = 800
HEIGHT = 600
FONT = cv2.FONT_HERSHEY_SIMPLEX





# ---------------------------------------------------------------------------
# ►►  The pupil‑detection functions below are **identical** to the originals
#     in OrloskyPupilDetector.py  citeturn6file0
# ---------------------------------------------------------------------------

def rotate_frame(frame: np.ndarray) -> np.ndarray:
    """Rotate the frame 180 degrees."""
    return cv2.rotate(frame, cv2.ROTATE_180)

def crop_to_aspect_ratio(image, width=640, height=480):
    current_height, current_width = image.shape[:2]
    desired_ratio = width / height
    current_ratio = current_width / current_height
    if current_ratio > desired_ratio:
        new_width = int(desired_ratio * current_height)
        offset = (current_width - new_width) // 2
        cropped = image[:, offset : offset + new_width]
    else:
        new_height = int(current_width / desired_ratio)
        offset = (current_height - new_height) // 2
        cropped = image[offset : offset + new_height, :]
    return cv2.resize(cropped, (width, height))

# (…the rest of the helper functions are copied verbatim; scroll ↓ if you want
#  to tweak the algorithm. Nothing below here touches the camera settings.)

# --- binary‑threshold helpers ------------------------------------------------

def apply_binary_threshold(image, darkest_pixel_value, added_threshold):
    _, out = cv2.threshold(image, darkest_pixel_value + added_threshold, 255, cv2.THRESH_BINARY_INV)
    return out

def get_darkest_area(image):
    ignore = 20
    step   = 10
    win    = 20
    skip   = 5
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Calculate center and bounds for 180x140 search area
    center_y = gray.shape[0] // 2
    center_x = gray.shape[1] // 2
    search_height = 160
    search_width = 210
    
    # Calculate bounds
    start_y = max(ignore, center_y - search_height//2)
    end_y = min(gray.shape[0] - ignore, center_y + search_height//2)
    start_x = max(ignore, center_x - search_width//2)
    end_x = min(gray.shape[1] - ignore, center_x + search_width//2)
    
    lowest_sum, darkest_pt = float("inf"), None
    for y in range(start_y, end_y, step):
        for x in range(start_x, end_x, step):
            block = gray[y : y + win : skip, x : x + win : skip]
            s = int(block.sum())
            if s < lowest_sum:
                lowest_sum, darkest_pt = s, (x + win // 2, y + win // 2)
    return darkest_pt

def mask_outside_square(img, center, size):
    x, y = center
    half = size // 2
    mask = np.zeros_like(img)
    mask[max(0, y - half) : min(img.shape[0], y + half), max(0, x - half) : min(img.shape[1], x + half)] = 255
    return cv2.bitwise_and(img, mask)

# (…ellipse helpers unchanged…)
# ---------------------------------------------------------------------------
#   ellipse utilities (filter_contours_by_area_and_return_largest, etc.)
# ---------------------------------------------------------------------------

def optimize_contours_by_angle(contours, image):
    if len(contours) < 1:
        return contours
    pts = np.concatenate(contours[0], axis=0)
    spacing = max(1, len(pts) // 25)
    centroid = np.mean(pts, axis=0)
    good = []
    for i in range(len(pts)):
        p = pts[i]
        p_prev = pts[i - spacing]
        p_next = pts[(i + spacing) % len(pts)]
        v1 = p_prev - p
        v2 = p_next - p
        denom = np.linalg.norm(v1) * np.linalg.norm(v2)
        if denom == 0:
            continue
        angle_ok = np.dot(v1, v2) / denom
        to_centroid = centroid - p
        if np.dot(to_centroid, (v1 + v2) / 2) >= np.cos(np.radians(60)):
            good.append(p)
    return np.array(good, dtype=np.int32).reshape((-1, 1, 2))

def filter_contours_by_area_and_return_largest(contours, pixel_thresh, ratio_thresh):
    best = None
    best_area = 0
    for c in contours:
        a = cv2.contourArea(c)
        if a < pixel_thresh:
            continue
        x, y, w, h = cv2.boundingRect(c)
        ratio = max(w / h, h / w)
        if ratio <= ratio_thresh and a > best_area:
            best, best_area = c, a
    return [best] if best is not None else []

# (check_contour_pixels, check_ellipse_goodness, fit_and_draw_ellipses – same as original)

def check_contour_pixels(contour, shape, dbg=False):
    if len(contour) < 5:
        return [0, 0, None]
    mask_contour = np.zeros(shape, np.uint8)
    cv2.drawContours(mask_contour, [contour], -1, 255, 1)
    ellipse = cv2.fitEllipse(contour)
    mask_thick = np.zeros(shape, np.uint8)
    mask_thin  = np.zeros(shape, np.uint8)
    cv2.ellipse(mask_thick, ellipse, 255, 10)
    cv2.ellipse(mask_thin,  ellipse, 255, 4)
    overlap_thick = cv2.bitwise_and(mask_contour, mask_thick)
    overlap_thin  = cv2.bitwise_and(mask_contour, mask_thin)
    abs_pix = int(np.sum(overlap_thick > 0))
    ratio = (np.sum(overlap_thin > 0) / np.sum(mask_contour > 0)) if np.sum(mask_contour > 0) else 0
    return [abs_pix, ratio, overlap_thin]

def check_ellipse_goodness(bin_img, contour, dbg=False):
    if len(contour) < 5:
        return [0, 0, 0]
    ellipse = cv2.fitEllipse(contour)
    mask = np.zeros_like(bin_img)
    cv2.ellipse(mask, ellipse, 255, -1)
    area = np.sum(mask == 255)
    filled = np.sum((bin_img == 255) & (mask == 255))
    if area == 0:
        return [0, 0, 0]
    skew = min(ellipse[1][0] / ellipse[1][1], ellipse[1][1] / ellipse[1][0])
    return [filled / area, 0, skew]

# ---------------------------------------------------------------------------
#   Frame‑level pipeline – untouched from original script
# ---------------------------------------------------------------------------

def process_frames(th_strict, th_med, th_relax, frame, gray, dark_pt, debug, render):
    best_score = 0
    best_contours = []
    best_img = None
    kernel = np.ones((5, 5), np.uint8)
    
    # Create a center mask (180x140 region)
    center_mask = np.zeros_like(gray)
    center_y = gray.shape[0] // 2
    center_x = gray.shape[1] // 2
    mask_height = 140
    mask_width = 180
    start_y = center_y - mask_height//2
    end_y = center_y + mask_height//2
    start_x = center_x - mask_width//2
    end_x = center_x + mask_width//2
    center_mask[start_y:end_y, start_x:end_x] = 255
    
    for thr_img in (th_relax, th_med, th_strict):
        # Apply center mask to thresholded image
        masked_thr = cv2.bitwise_and(thr_img, center_mask)
        dil = cv2.dilate(masked_thr, kernel, iterations=2)
        contours, _ = cv2.findContours(dil, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        biggest = filter_contours_by_area_and_return_largest(contours, 36, 3)
        if biggest and len(biggest[0]) > 5:
            g = check_ellipse_goodness(dil, biggest[0])
            abs_pix, ratio, ov = check_contour_pixels(biggest[0], dil.shape)
            score = g[0] * abs_pix * abs_pix * ratio
            if score > best_score:
                best_score, best_contours, best_img = score, biggest, dil
    if not best_contours:
        return (0, 0), frame  # no ellipse this frame
    opt = [optimize_contours_by_angle(best_contours, gray)]
    if len(opt[0]) <= 5:
        return (0, 0), frame
    ellipse = cv2.fitEllipse(opt[0])
    cv2.ellipse(frame, ellipse, (55, 255, 0), 2)
    cx, cy = map(int, ellipse[0])
    cv2.circle(frame, (cx, cy), 3, (255, 255, 0), -1)
    if render:
        cv2.imshow("Pupil", frame)
    return (cx, cy), frame

# Global variable to store last known pupil ellipse
last_pupil_ellipse = None
# Global variable to store reference point
reference_point = None

def process_frame(frame, debug=False, render=False):
    global last_pupil_ellipse, reference_point
    frame = crop_to_aspect_ratio(frame)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Define ROI parameters
    ROI_W = 310
    ROI_H = 200
    center_y = frame.shape[0] // 2
    center_x = frame.shape[1] // 2
    
    # Calculate ROI bounds with 40 pixel right offset
    x0 = max(0, center_x - ROI_W//2 + 80)  # Added 40 pixel offset to the right
    y0 = max(0, center_y - ROI_H//2 + 70)
    x0 = min(x0, frame.shape[1] - ROI_W)
    y0 = min(y0, frame.shape[0] - ROI_H)
    
    # Extract ROI
    roi = frame[y0:y0+ROI_H, x0:x0+ROI_W]
    gray_roi = gray[y0:y0+ROI_H, x0:x0+ROI_W]
    
    # Get darkest point in ROI
    dark_pt_roi = get_darkest_area(roi)
    if dark_pt_roi is None:
        return (0, 0), frame
        
    dark_val = gray_roi[dark_pt_roi[1], dark_pt_roi[0]]
    
    # Create thresholded images for ROI
    th_strict = apply_binary_threshold(gray_roi, dark_val, 5)
    th_med = apply_binary_threshold(gray_roi, dark_val, 15)
    th_relax = apply_binary_threshold(gray_roi, dark_val, 25)
    
    # Process frames for pupil detection in ROI
    (cx_roi, cy_roi), _ = process_frames(th_strict, th_med, th_relax, roi, gray_roi, dark_pt_roi, debug, False)
    
    # Detect the iris as a grey circle via HoughCircles
    blur = cv2.medianBlur(gray_roi, 7)
    # Only consider mid-grey intensities (e.g. iris) by pre-masking
    grey_mask = cv2.inRange(blur, 40, 160)
    grey_blur = cv2.bitwise_and(blur, blur, mask=grey_mask)
    
    # Hough parameters tuned for iris scale
    circles = cv2.HoughCircles(
        grey_blur,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=ROI_H/2,
        param1=50,
        param2=30,
        minRadius=ROI_W//8,
        maxRadius=ROI_W//2,
    )
    
    iris = None
    if circles is not None:
        # take the circle closest to ROI center
        circles = np.round(circles[0]).astype(int)
        cx0, cy0 = ROI_W//2, ROI_H//2
        iris = min(circles, key=lambda c: (c[0]-cx0)**2 + (c[1]-cy0)**2)
    
    # Draw ROI rectangle
    if render:
        cv2.rectangle(frame, (x0,y0), (x0+ROI_W,y0+ROI_H), (255,0,0), 1)
    
    # Draw pupil
    ellipse_drawn = False
    ellipse_center = None
    if (cx_roi, cy_roi) != (0,0):
        cx_full = cx_roi + x0
        cy_full = cy_roi + y0
        th_imgs = [th_relax, th_med, th_strict]
        best_score = 0
        best_contours = []
        for thr_img in th_imgs:
            kernel = np.ones((5, 5), np.uint8)
            center_mask = np.zeros_like(gray_roi)
            center_y_roi = gray_roi.shape[0] // 2
            center_x_roi = gray_roi.shape[1] // 2
            mask_height = 140
            mask_width = 180
            start_y = center_y_roi - mask_height//2
            end_y = center_y_roi + mask_height//2
            start_x = center_x_roi - mask_width//2
            end_x = center_x_roi + mask_width//2
            center_mask[start_y:end_y, start_x:end_x] = 255
            masked_thr = cv2.bitwise_and(thr_img, center_mask)
            dil = cv2.dilate(masked_thr, kernel, iterations=2)
            contours, _ = cv2.findContours(dil, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            biggest = filter_contours_by_area_and_return_largest(contours, 36, 3)
            if biggest and len(biggest[0]) > 5:
                g = check_ellipse_goodness(dil, biggest[0])
                abs_pix, ratio, ov = check_contour_pixels(biggest[0], dil.shape)
                score = g[0] * abs_pix * abs_pix * ratio
                if score > best_score:
                    best_score, best_contours = score, biggest
        if best_contours and len(best_contours[0]) > 5:
            opt = [optimize_contours_by_angle(best_contours, gray_roi)]
            if len(opt[0]) > 5:
                ellipse = cv2.fitEllipse(opt[0])
                ellipse_offset = ((ellipse[0][0] + x0, ellipse[0][1] + y0), ellipse[1], ellipse[2])
                # Draw blue circle and dot instead of green ellipse and dot
                center = (int(ellipse_offset[0][0]), int(ellipse_offset[0][1]))
                radius = int(max(ellipse_offset[1]) / 2) # Approximate radius from major axis
                cv2.circle(frame, center, radius, (255,0,0), 2) # Blue circle
                cv2.circle(frame, center, 4, (255,0,0), -1) # Blue dot
                last_pupil_ellipse = ellipse_offset
                ellipse_drawn = True
                ellipse_center = center
    else:
        cx_full, cy_full = 0, 0
    # If no ellipse was drawn, draw the last known ellipse as a blue circle/dot
    if not ellipse_drawn and last_pupil_ellipse is not None:
        center = (int(last_pupil_ellipse[0][0]), int(last_pupil_ellipse[0][1]))
        radius = int(max(last_pupil_ellipse[1]) / 2) # Approximate radius from major axis
        cv2.circle(frame, center, radius, (255,0,0), 2) # Blue circle
        cv2.circle(frame, center, 4, (255,0,0), -1) # Blue dot
        ellipse_center = center
    # Show deviation from reference point if set
    if reference_point is not None and ellipse_center is not None:
        # Draw red dot at reference point
        cv2.circle(frame, reference_point, 4, (0,0,255), -1)
        
        dy = ellipse_center[1] - reference_point[1]
        dx = ellipse_center[0] - reference_point[0]
        # Invert y deviation and show x first
        cv2.putText(frame, f"Deviation (x,y): ({dx}, {-dy})", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
    
    # Draw iris (REMOVED BLUE CIRCLE)
    # if iris is not None:
    #     ix_full = iris[0] + x0
    #     iy_full = iris[1] + y0
    #     iradius = iris[2]
    #     cv2.circle(frame, (ix_full, iy_full), iradius, (255,0,0), 2)
    
    if render:
        cv2.imshow("Pupil", frame)
    
    return (cx_full, cy_full), frame








# TCP Config
HOST = '127.0.0.1'
PORT = 65432
smoothing_window_size = 10
pupil_coords_history = []

# Check test mode flag from command-line argument


if not TEST_MODE:
    cap_pupil = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    if not cap_pupil.isOpened():
        print("[EYE] Could not open camera.")
        exit(1)

print(f"[EYE] Eye tracker running. Sending gaze data to: {HOST}:{PORT} | Test mode: {TEST_MODE}")


while True:
    if TEST_MODE:
        time.sleep(0.1)
        pupil_coords = (random.randint(-400, 400), random.randint(-400, 400))
    else:
        time.sleep(0.01)
        ret_pupil, frame_pupil = cap_pupil.read()
        if not ret_pupil:
            break

        pupil_coords, frame_pupil = process_frame(frame_pupil, render=False)
        frame_pupil_tracked = frame_pupil.copy()

    if pupil_coords != (0, 0):
        pupil_coords_history.append(pupil_coords)
        if len(pupil_coords_history) > smoothing_window_size:
            pupil_coords_history.pop(0)

    smoothed = (0, 0)
    if pupil_coords_history:
        sum_x = sum(p[0] for p in pupil_coords_history)
        sum_y = sum(p[1] for p in pupil_coords_history)
        smoothed = (int(sum_x / len(pupil_coords_history)), int(sum_y / len(pupil_coords_history)))

        # Auto-set reference point once
        if reference_point is None and smoothed != (0, 0):
            reference_point = smoothed
            print(f"[AUTO] Reference point set to: {reference_point}")

        # Compute raw deviation
        dx = smoothed[0] - reference_point[0]
        dy = smoothed[1] - reference_point[1]

        # Send raw deviation to GUI controller
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((HOST, PORT))
            sock.sendall(f"{dx},{dy}".encode("utf-8"))
        except Exception as e:
            print(f"[SOCKET ERROR] {type(e).__name__}: {e}")
        finally:
            sock.close()

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

if not TEST_MODE:
    cap_pupil.release()
cv2.destroyAllWindows()