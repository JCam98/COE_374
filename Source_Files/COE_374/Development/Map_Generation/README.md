# Basic Stitching Algorithm for Aerial Images

## stitching.py

A basic stitching algorithm that that utilizes SIFT, FLANN, RANSAC, and homography to create a stitched map using an input directory of aerial images.

To run the script:

```bash
python3 stitching.py <image_dir> <key_frame> <output_dir>
```

## resize.py

Quick script to resize a directory of images

```bash
python resize.py <image_dir> <output_dir> <% size of input image>
```

## png.py

Might not even use, just a quick script to test what the maps might look like as a transparent png file instead of a jpg with a black background.

