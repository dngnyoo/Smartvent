#!/usr/bin/env python3
"""
Chevron Display Node - Command Velocity Based (Pre-initialized Pygame)

This ROS2 node initializes pygame at startup and keeps it running,
eliminating startup delay when displaying chevrons.

Integration with GoPiGo3:
- Forward motion (linear.x > 0) → RIGHT chevrons
- Backward motion (linear.x < 0) → LEFT chevrons
- Left turn (angular.z > 0) → UP chevrons
- Right turn (angular.z < 0) → DOWN chevrons
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
import time
import os
import pygame
import math


class ChevronCmdVelNode(Node):
    """
    ROS2 node that displays chevrons based on /cmd_vel commands.
    Pygame is initialized once at startup for instant response.
    """
    
    def __init__(self):
        super().__init__('chevron_cmd_vel_node')
        
        # Declare parameters
        self.declare_parameter('linear_threshold', 0.05)
        self.declare_parameter('angular_threshold', 0.1)
        self.declare_parameter('display_duration', 2.0)
        self.declare_parameter('chevron_speed', 220.0)
        self.declare_parameter('chevron_spacing', 220.0)
        self.declare_parameter('priority_mode', 'linear')
        
        # Get parameters
        self.linear_threshold = self.get_parameter('linear_threshold').value
        self.angular_threshold = self.get_parameter('angular_threshold').value
        self.display_duration = self.get_parameter('display_duration').value
        self.chevron_speed = self.get_parameter('chevron_speed').value
        self.chevron_spacing = self.get_parameter('chevron_spacing').value
        self.priority_mode = self.get_parameter('priority_mode').value
        
        # Initialize pygame ONCE at startup
        self.init_pygame()
        
        # Display state
        self.current_direction = None
        self.display_start_time = 0
        self.offset = 0.0
        self.last_update_time = time.time()
        
        # Subscribe to cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Timer for continuous drawing at 60 FPS
        self.timer = self.create_timer(1.0 / 60.0, self.draw_loop)
        
        self.get_logger().info('Chevron Display Node started (pygame pre-initialized)')
        self.get_logger().info(f'Parameters:')
        self.get_logger().info(f'  Linear threshold: {self.linear_threshold} m/s')
        self.get_logger().info(f'  Angular threshold: {self.angular_threshold} rad/s')
        self.get_logger().info(f'  Display duration: {self.display_duration} s')
        self.get_logger().info('Listening to /cmd_vel...')
    
    def init_pygame(self):
        """Initialize pygame once at startup"""
        os.environ.setdefault("SDL_VIDEO_FULLSCREEN_HEAD", "0")
        os.environ.setdefault("DISPLAY", ":0")
        
        pygame.init()
        
        self.screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
        self.screen_width, self.screen_height = self.screen.get_size()
        pygame.display.set_caption("Chevron Display")
        
        self.clock = pygame.time.Clock()
        
        # Chevron size
        base_size = min(self.screen_width, self.screen_height)
        self.chevron_width = base_size * 0.12
        self.chevron_height = base_size * 0.08
        
        self.get_logger().info(f'Pygame initialized: {self.screen_width}x{self.screen_height}')
    
    def cmd_vel_callback(self, msg):
        """Process incoming velocity commands"""
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Determine which direction to display
        direction = self.determine_direction(linear_x, angular_z)
        
        if direction:
            # Only update if direction changed or display expired
            now = time.time()
            if (direction != self.current_direction or 
                now - self.display_start_time > self.display_duration):
                
                self.current_direction = direction
                self.display_start_time = now
                self.offset = 0.0
                
                self.get_logger().info(
                    f'cmd_vel: linear_x={linear_x:.3f}, angular_z={angular_z:.3f} -> {direction.upper()}'
                )
    
    def determine_direction(self, linear_x, angular_z):
        """Determine which chevron direction to display"""
        linear_active = abs(linear_x) > self.linear_threshold
        angular_active = abs(angular_z) > self.angular_threshold
        
        if self.priority_mode == 'linear':
            if linear_active:
                return 'right' if linear_x > 0 else 'left'
            elif angular_active:
                return 'up' if angular_z > 0 else 'down'
        
        elif self.priority_mode == 'angular':
            if angular_active:
                return 'up' if angular_z > 0 else 'down'
            elif linear_active:
                return 'right' if linear_x > 0 else 'left'
        
        elif self.priority_mode == 'both':
            if linear_active and angular_active:
                linear_magnitude = abs(linear_x) / self.linear_threshold
                angular_magnitude = abs(angular_z) / self.angular_threshold
                
                if linear_magnitude > angular_magnitude:
                    return 'right' if linear_x > 0 else 'left'
                else:
                    return 'up' if angular_z > 0 else 'down'
            elif linear_active:
                return 'right' if linear_x > 0 else 'left'
            elif angular_active:
                return 'up' if angular_z > 0 else 'down'
        
        return None
    
    def draw_loop(self):
        """Main drawing loop - runs continuously at 60 FPS"""
        # Handle pygame events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                rclpy.shutdown()
                return
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    rclpy.shutdown()
                    return
        
        # Check if current display should expire
        if self.current_direction:
            elapsed = time.time() - self.display_start_time
            if elapsed > self.display_duration:
                self.current_direction = None
        
        # Update animation offset
        now = time.time()
        dt = now - self.last_update_time
        self.last_update_time = now
        
        if self.current_direction:
            self.offset += self.chevron_speed * dt
            if self.offset >= self.chevron_spacing:
                self.offset -= self.chevron_spacing
        
        # Draw
        self.screen.fill((0, 0, 0))  # Black background
        
        if self.current_direction:
            self.draw_chevrons(self.current_direction)
        
        pygame.display.flip()
        self.clock.tick(60)
    
    def draw_chevrons(self, direction):
        """Draw chevrons in the specified direction"""
        color = (255, 255, 255)
        
        # Based on actual chevron file contents:
        # chevron_right.py = UP chevrons (vertical stream)
        # chevron_left.py = DOWN chevrons (vertical stream)
        # chevron_up.py = LEFT chevrons (horizontal stream)
        # chevron_down.py = RIGHT chevrons (horizontal stream)
        
        if direction in ('up', 'down'):
            # Vertical stream (chevron_right.py and chevron_left.py)
            x = self.screen_width / 2
            count = int(self.screen_height / self.chevron_spacing) + 3
            # 'up' uses chevron_right.py which moves up (negative offset)
            # 'down' uses chevron_left.py which moves down (positive offset)
            sign = -1 if direction == 'up' else 1
            
            for i in range(-1, count):
                y = i * self.chevron_spacing + sign * self.offset
                if -self.chevron_spacing <= y <= self.screen_height + self.chevron_spacing:
                    self.draw_chevron(x, y, direction, color)
        else:
            # Horizontal stream (chevron_up.py and chevron_down.py)
            y = self.screen_height / 2
            count = int(self.screen_width / self.chevron_spacing) + 3
            # 'left' uses chevron_up.py which moves left (negative offset)
            # 'right' uses chevron_down.py which moves right (positive offset)
            sign = -1 if direction == 'left' else 1
            
            for i in range(-1, count):
                x = i * self.chevron_spacing + sign * self.offset
                if -self.chevron_spacing <= x <= self.screen_width + self.chevron_spacing:
                    self.draw_chevron(x, y, direction, color)
    
    def draw_chevrons(self, direction):
        """Draw chevrons matching YOUR file names exactly
        
        Robot motion → File to use:
        - Forward → chevron_right.py (draws UP chevrons moving UP)
        - Backward → chevron_left.py (draws DOWN chevrons moving DOWN) 
        - Turn Left → chevron_up.py (draws LEFT chevrons moving LEFT)
        - Turn Right → chevron_down.py (draws RIGHT chevrons moving RIGHT)
        """
        color = (255, 255, 255)
        
        # chevron_right.py: vertical UP chevrons
        if direction == 'right':
            x = self.screen_width / 2
            count = int(self.screen_height / self.chevron_spacing) + 3
            for i in range(-1, count):
                y = i * self.chevron_spacing - self.offset  # Moving up
                if -self.chevron_spacing <= y <= self.screen_height + self.chevron_spacing:
                    self.draw_up_chevron(x, y, color)
        
        # chevron_left.py: vertical DOWN chevrons  
        elif direction == 'left':
            x = self.screen_width / 2
            count = int(self.screen_height / self.chevron_spacing) + 3
            for i in range(-1, count):
                y = i * self.chevron_spacing + self.offset  # Moving down
                if -self.chevron_spacing <= y <= self.screen_height + self.chevron_spacing:
                    self.draw_down_chevron(x, y, color)
        
        # chevron_up.py: horizontal LEFT chevrons
        elif direction == 'up':
            y = self.screen_height / 2
            count = int(self.screen_width / self.chevron_spacing) + 3
            for i in range(-1, count):
                x = i * self.chevron_spacing - self.offset  # Moving left
                if -self.chevron_spacing <= x <= self.screen_width + self.chevron_spacing:
                    self.draw_left_chevron(x, y, color)
        
        # chevron_down.py: horizontal RIGHT chevrons
        elif direction == 'down':
            y = self.screen_height / 2
            count = int(self.screen_width / self.chevron_spacing) + 3
            for i in range(-1, count):
                x = i * self.chevron_spacing + self.offset  # Moving right
                if -self.chevron_spacing <= x <= self.screen_width + self.chevron_spacing:
                    self.draw_right_chevron(x, y, color)
    
    def draw_right_chevron(self, cx, cy, color):
        """RIGHT - from chevron_right.py"""
        points = [
            (cx, cy - self.chevron_height / 2),
            (cx + self.chevron_width / 2, cy + self.chevron_height / 2),
            (cx - self.chevron_width / 2, cy + self.chevron_height / 2),
        ]
        pygame.draw.polygon(self.screen, color, points)
    
    def draw_left_chevron(self, cx, cy, color):
        """LEFT - from chevron_left.py"""
        points = [
            (cx, cy + self.chevron_height / 2),
            (cx + self.chevron_width / 2, cy - self.chevron_height / 2),
            (cx - self.chevron_width / 2, cy - self.chevron_height / 2),
        ]
        pygame.draw.polygon(self.screen, color, points)
    
    def draw_up_chevron(self, cx, cy, color):
        """UP - from chevron_up.py"""
        points = [
            (cx + self.chevron_width / 2, cy - self.chevron_height / 2),
            (cx - self.chevron_width / 2, cy),
            (cx + self.chevron_width / 2, cy + self.chevron_height / 2),
        ]
        pygame.draw.polygon(self.screen, color, points)
    
    def draw_down_chevron(self, cx, cy, color):
        """DOWN - Corrected to point down (v) based on reference"""
        points = [
            # 1. 아래쪽 꼭짓점 (Tip)
            (cx, cy + self.chevron_height / 2),
            # 2. 오른쪽 위 모서리 (Top Right)
            (cx + self.chevron_width / 2, cy - self.chevron_height / 2),
            # 3. 왼쪽 위 모서리 (Top Left)
            (cx - self.chevron_width / 2, cy - self.chevron_height / 2),
        ]
        
        # 실제 화면에 그리는 함수 (self.screen이 정의되어 있다고 가정)
        pygame.draw.polygon(self.screen, color, points)
    
    def destroy_node(self):
        """Cleanup"""
        pygame.quit()
        super().destroy_node()


def main(args=None):
    """Main function to run the node"""
    rclpy.init(args=args)
    
    node = ChevronCmdVelNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        node.get_logger().info('Shutting down chevron display node')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
