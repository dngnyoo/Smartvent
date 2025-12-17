#!/usr/bin/env python3
"""
Raspberry Pi Camera V2 Test Script
Tests: Detection, Capture, and Image Quality
"""

import sys
import cv2
import numpy as np
from datetime import datetime

def test_camera():
    print("=" * 60)
    print("Raspberry Pi Camera V2 Test")
    print("=" * 60)
    
    # Try common camera indices
    camera_indices = [0, 1, 10]
    camera = None
    working_index = None
    
    print("\n[1/5] Detecting camera...")
    for idx in camera_indices:
        print(f"   Trying /dev/video{idx}...", end="")
        cap = cv2.VideoCapture(idx)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret and frame is not None:
                print(f" ✅ Found!")
                camera = cap
                working_index = idx
                break
            cap.release()
        print(" ❌")
    
    if camera is None:
        print("\n❌ No camera found!")
        print("\nTroubleshooting:")
        print("1. Check camera connection: vcgencmd get_camera")
        print("2. Enable camera: sudo raspi-config")
        print("3. Reboot after enabling camera")
        return False
    
    print(f"✅ Camera detected at /dev/video{working_index}")
    
    # Get camera properties
    print("\n[2/5] Reading camera properties...")
    width = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(camera.get(cv2.CAP_PROP_FPS))
    print(f"   Resolution: {width}x{height}")
    print(f"   FPS: {fps}")
    
    # Capture test image
    print("\n[3/5] Capturing test image...")
    ret, frame = camera.read()
    if not ret:
        print("❌ Failed to capture image")
        camera.release()
        return False
    
    print(f"✅ Image captured: {frame.shape}")
    
    # Save test image
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"/home/ubuntu/camera_test_{timestamp}.jpg"
    cv2.imwrite(filename, frame)
    print(f"✅ Image saved: {filename}")
    
    # Analyze image quality
    print("\n[4/5] Analyzing image quality...")
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    brightness = np.mean(gray)
    contrast = np.std(gray)
    
    print(f"   Brightness: {brightness:.1f}/255")
    print(f"   Contrast: {contrast:.1f}")
    
    if brightness < 30:
        print("   ⚠️  Image too dark")
    elif brightness > 225:
        print("   ⚠️  Image too bright")
    else:
        print("   ✅ Brightness OK")
    
    if contrast < 20:
        print("   ⚠️  Low contrast")
    else:
        print("   ✅ Contrast OK")
    
    # Test continuous capture
    print("\n[5/5] Testing continuous capture (5 frames)...")
    for i in range(5):
        ret, frame = camera.read()
        if ret:
            print(f"   Frame {i+1}: {frame.shape} ✅")
        else:
            print(f"   Frame {i+1}: Failed ❌")
    
    camera.release()
    
    print("\n" + "=" * 60)
    print("🎉 Camera Test Complete!")
    print(f"📸 Test image saved at: {filename}")
    print("=" * 60)
    
    return True

if __name__ == "__main__":
    try:
        test_camera()
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
