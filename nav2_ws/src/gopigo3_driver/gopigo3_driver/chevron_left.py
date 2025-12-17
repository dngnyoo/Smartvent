#!/usr/bin/env python3
"""
Chevron Display - left Direction
Standalone module for displaying left-pointing chevrons moving left.

Usage:
    python chevron_left.py
    
Or import and use:
    from chevron_left import display
    display()
"""

import os
import time
import pygame

os.environ["DISPLAY"] = ":0"

def display(speed: float = 220.0, spacing: float = 220.0, duration: float = None):
    """
    Display left-pointing chevrons moving left.
    
    Args:
        speed: Movement speed in pixels per second (default: 220)
        spacing: Distance between chevrons in pixels (default: 220)
        duration: How long to display in seconds (default: None = until ESC pressed)
    
    Example:
        display()  # Run until ESC
        display(speed=300, duration=10.0)  # 10 seconds at 300 px/s
    """
    # Set display for multi-monitor setups
    os.environ.setdefault("SDL_VIDEO_FULLSCREEN_HEAD", "0")
    
    pygame.init()
    
    # Fullscreen display
    screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
    screen_width, screen_height = screen.get_size()
    pygame.display.set_caption("Chevron Display - left")
    
    clock = pygame.time.Clock()
    
    # Chevron size (scales with screen)
    base_size = min(screen_width, screen_height)
    chevron_width = base_size * 0.12
    chevron_height = base_size * 0.08
    
    # Animation state
    offset = 0.0
    
    # Timing
    running = True
    last_time = time.time()
    start_time = time.time()
    
    while running:
        # Event handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
        
        # Check duration
        if duration is not None:
            if time.time() - start_time >= duration:
                running = False
        
        # Update timing
        now = time.time()
        dt = now - last_time
        last_time = now
        
        # Update animation offset (moving left)
        offset += speed * dt
        if offset >= spacing:
            offset -= spacing
        
        # Draw frame
        screen.fill((0, 0, 0))  # Black background
        
        # Draw horizontal stream of left-pointing chevrons
        x = screen_width / 2
        count = int(screen_height / spacing) + 3
        
        for i in range(-1, count):
            y = i * spacing + offset  # Moving left (positive offset)
            
            if -spacing <= y <= screen_height + spacing:
                # Draw left-pointing chevron "v"
                points = [
                    (x, y + chevron_height / 2),
                    (x + chevron_width / 2, y - chevron_height / 2),
                    (x - chevron_width / 2, y - chevron_height / 2),
                ]
                pygame.draw.polygon(screen, (255, 255, 255), points)
        
        pygame.display.flip()
        clock.tick(60)
    
    pygame.quit()


if __name__ == "__main__":
    display()
