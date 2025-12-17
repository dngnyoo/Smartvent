#!/usr/bin/env python3
"""
Chevron Display Node - Command Velocity Based (Pre-initialized Pygame)

Pygame initialized at startup for instant response.
Display priority:
  1. Gas hazard (when gas detected)
  2. Chevrons (when moving)
  3. Grid pattern with blinking anchors (when idle)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import time
import os
import pygame
import numpy as np


class ChevronCmdVelNode(Node):
    
    def __init__(self):
        super().__init__('chevron_cmd_vel_node')
        
        # Declare parameters
        self.declare_parameter('linear_threshold', 0.05)
        self.declare_parameter('angular_threshold', 0.1)
        self.declare_parameter('display_duration', 2.0)
        self.declare_parameter('chevron_speed', 220.0)
        self.declare_parameter('chevron_spacing', 220.0)
        self.declare_parameter('priority_mode', 'linear')
        self.declare_parameter('gas_threshold', 75.0)
        
        # Grid pattern parameters
        self.declare_parameter('grid_block_size', 32)
        self.declare_parameter('grid_anchor_spacing', 4)
        
        # Get parameters
        self.linear_threshold = self.get_parameter('linear_threshold').value
        self.angular_threshold = self.get_parameter('angular_threshold').value
        self.display_duration = self.get_parameter('display_duration').value
        self.chevron_speed = self.get_parameter('chevron_speed').value
        self.chevron_spacing = self.get_parameter('chevron_spacing').value
        self.priority_mode = self.get_parameter('priority_mode').value
        self.gas_threshold = self.get_parameter('gas_threshold').value
        self.grid_block_size = self.get_parameter('grid_block_size').value
        self.grid_anchor_spacing = self.get_parameter('grid_anchor_spacing').value
        
        # Initialize pygame ONCE at startup
        self.init_pygame()
        
        # Chevron display state
        self.current_direction = None
        self.display_start_time = 0
        self.offset = 0.0
        self.last_update_time = time.time()
        
        # Gas hazard state
        self.gas_level = 0.0
        self.show_gas_hazard = False
        
        # Grid pattern state
        self.grid_frame_idx = 0
        self.init_grid_pattern()
        
        # Subscribe to cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Subscribe to gas_level
        self.gas_subscription = self.create_subscription(
            Float32,
            'gas_level',
            self.gas_callback,
            10
        )
        
        # Timer for continuous drawing at 60 FPS
        self.timer = self.create_timer(1.0 / 60.0, self.draw_loop)
        
        self.get_logger().info('Chevron Display Node started (pygame pre-initialized)')
        self.get_logger().info('Display modes: Gas Hazard > Chevrons > Grid Pattern')
        self.get_logger().info(f'Gas threshold: {self.gas_threshold}')
    
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
        
        # Font for gas hazard text
        self.font = pygame.font.SysFont("Arial", int(self.screen_height * 0.1))
        self.level_font = pygame.font.SysFont("Arial", int(self.screen_height * 0.05))
        
        self.get_logger().info(f'Pygame initialized: {self.screen_width}x{self.screen_height}')
    
    def init_grid_pattern(self):
        """Initialize grid pattern with blinking anchors"""
        # Grid pattern constants (from test_projection.py)
        self.CODE_LEN = 7
        self.CODES = [
            0b1010011, 0b1100101, 0b0110110, 0b1110001,
            0b1001110, 0b0101011, 0b0011101
        ]
        self.SYMBOL_HOLD = 3  # frames per symbol
        self.ANCHOR_SIZE_RATIO = 0.35
        
        # Precompute code bits
        self.CODE_BITS = np.stack([
            np.array([(c >> i) & 1 for i in range(self.CODE_LEN)], dtype=np.uint8)
            for c in self.CODES
        ], axis=0)
        
        # Build grid layout
        self.rebuild_grid()
        
        self.get_logger().info(f'Grid pattern initialized: {self.grid_w}x{self.grid_h} cells')
    
    def rebuild_grid(self):
        """Build or rebuild grid pattern"""
        # Calculate grid dimensions
        self.grid_h = max(2, self.screen_height // self.grid_block_size)
        self.grid_w = max(2, self.screen_width // self.grid_block_size)
        
        # Anchor mask (every Nth cell)
        self.anchor_mask = np.zeros((self.grid_h, self.grid_w), np.uint8)
        self.anchor_mask[
            (np.arange(self.grid_h)[:, None] % self.grid_anchor_spacing == 0) &
            (np.arange(self.grid_w)[None, :] % self.grid_anchor_spacing == 0)
        ] = 1
        
        # Random code assignments and phases
        rng = np.random.default_rng(123)
        self.code_idx = rng.integers(0, len(self.CODES), size=(self.grid_h, self.grid_w), dtype=np.int32)
        self.phase = rng.integers(0, self.CODE_LEN, size=(self.grid_h, self.grid_w), dtype=np.int32)
        
        # Build base grid surface (inverted: light lines on dark bg)
        self.grid_surface = self.build_grid_surface()
    
    def build_grid_surface(self):
        """Build the base grid surface (light lines on dark background)"""
        surface = pygame.Surface((self.screen_width, self.screen_height))
        surface.fill((0, 0, 0))  # Black background
        
        LINE_THIN = 1
        LINE_THICK = 2
        MAJOR_EVERY = 4
        
        # Draw vertical lines
        for ix in range(0, self.screen_width + 1, self.grid_block_size):
            major = ((ix // self.grid_block_size) % MAJOR_EVERY == 0)
            width = LINE_THICK if major else LINE_THIN
            pygame.draw.line(surface, (255, 255, 255), (ix, 0), (ix, self.screen_height), width)
        
        # Draw horizontal lines
        for iy in range(0, self.screen_height + 1, self.grid_block_size):
            major = ((iy // self.grid_block_size) % MAJOR_EVERY == 0)
            width = LINE_THICK if major else LINE_THIN
            pygame.draw.line(surface, (255, 255, 255), (0, iy), (self.screen_width, iy), width)
        
        return surface
    
    def gas_callback(self, msg: Float32):
        """Handle incoming gas level data"""
        self.gas_level = msg.data
        
        if self.gas_level >= self.gas_threshold:
            if not self.show_gas_hazard:
                self.show_gas_hazard = True
                self.get_logger().warn(f'🚨 GAS HAZARD! Level: {self.gas_level:.1f}')
        else:
            if self.show_gas_hazard:
                self.show_gas_hazard = False
                self.get_logger().info(f'✅ Gas cleared. Level: {self.gas_level:.1f}')
    
    def cmd_vel_callback(self, msg):
        """Process incoming velocity commands"""
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        direction = self.determine_direction(linear_x, angular_z)
        
        if direction:
            now = time.time()
            if (direction != self.current_direction or 
                now - self.display_start_time > self.display_duration):
                
                self.current_direction = direction
                self.display_start_time = now
                self.offset = 0.0
                
                self.get_logger().info(f'Direction: {direction.upper()}')
    
    def determine_direction(self, linear_x, angular_z):
        """Determine which chevron direction to display"""
        linear_active = abs(linear_x) > self.linear_threshold
        angular_active = abs(angular_z) > self.angular_threshold
        
        if linear_active:
            return 'up' if linear_x > 0 else 'down'
        
        if angular_active:
            return 'left' if angular_z > 0 else 'right'
        
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
        
        # Check if current chevron display should expire
        if self.current_direction:
            elapsed = time.time() - self.display_start_time
            if elapsed > self.display_duration:
                self.current_direction = None
        
        # Update animation offset for chevrons
        now = time.time()
        dt = now - self.last_update_time
        self.last_update_time = now
        
        if self.current_direction:
            self.offset += self.chevron_speed * dt
            if self.offset >= self.chevron_spacing:
                self.offset -= self.chevron_spacing
        
        # Draw - PRIORITY: Gas > Chevrons > Grid
        self.screen.fill((0, 0, 0))
        
        if self.show_gas_hazard:
            # PRIORITY 1: Show gas hazard
            self.draw_gas_hazard()
        elif self.current_direction == 'up':
            # PRIORITY 2: Show chevrons
            self.draw_right_chevrons()
        elif self.current_direction == 'down':
            self.draw_left_chevrons()
        elif self.current_direction == 'left':
            self.draw_up_chevrons()
        elif self.current_direction == 'right':
            self.draw_down_chevrons()
        else:
            # PRIORITY 3: Show grid pattern (idle)
            self.draw_grid_pattern()
        
        pygame.display.flip()
        self.clock.tick(60)
        self.grid_frame_idx += 1
    
    def draw_grid_pattern(self):
        """Draw grid pattern with blinking anchors"""
        # Blit the base grid
        self.screen.blit(self.grid_surface, (0, 0))
        
        # Current symbol time index
        t = (self.grid_frame_idx // self.SYMBOL_HOLD) % self.CODE_LEN
        
        # Anchor sub-rect size/offset
        a = max(2, int(self.grid_block_size * self.ANCHOR_SIZE_RATIO))
        off = (self.grid_block_size - a) // 2
        
        # Draw blinking anchors
        ys, xs = np.nonzero(self.anchor_mask)
        for y, x in zip(ys, xs):
            bit = int(self.CODE_BITS[self.code_idx[y, x], (t + self.phase[y, x]) % self.CODE_LEN])
            
            # Sub-rect pixel coords
            x0 = x * self.grid_block_size + off
            y0 = y * self.grid_block_size + off
            x1 = min(x0 + a, self.screen_width)
            y1 = min(y0 + a, self.screen_height)
            
            # Light blink on dark background
            color = (255, 255, 255) if bit == 1 else (0, 0, 0)
            pygame.draw.rect(self.screen, color, (x0, y0, x1 - x0, y1 - y0))
    
    def draw_gas_hazard(self):
        """Draw gas hazard warning"""
        w, h = self.screen_width, self.screen_height
        
        # Yellow warning triangle
        top = (w / 2, h * 0.1)
        left = (w * 0.15, h * 0.8)
        right = (w * 0.85, h * 0.8)
        
        pygame.draw.polygon(self.screen, (255, 255, 0), [top, left, right])
        pygame.draw.polygon(self.screen, (255, 255, 255), [top, left, right], width=10)
        
        # White text
        text_surface = self.font.render("GAS HAZARD", True, (255, 255, 255))
        text_rect = text_surface.get_rect()
        text_rect.center = (w / 2, h * 0.85)
        self.screen.blit(text_surface, text_rect)
        
        # Show current level
        level_text = self.level_font.render(f"Level: {self.gas_level:.1f}", True, (255, 255, 255))
        level_rect = level_text.get_rect()
        level_rect.center = (w / 2, h * 0.92)
        self.screen.blit(level_text, level_rect)
    
    def draw_right_chevrons(self):
        """RIGHT - vertical stream, UP-pointing chevrons moving UP"""
        x = self.screen_width / 2
        count = int(self.screen_height / self.chevron_spacing) + 3
        
        for i in range(-1, count):
            y = i * self.chevron_spacing - self.offset
            
            if -self.chevron_spacing <= y <= self.screen_height + self.chevron_spacing:
                points = [
                    (x, y - self.chevron_height / 2),
                    (x + self.chevron_width / 2, y + self.chevron_height / 2),
                    (x - self.chevron_width / 2, y + self.chevron_height / 2),
                ]
                pygame.draw.polygon(self.screen, (255, 255, 255), points)
    
    def draw_left_chevrons(self):
        """LEFT - vertical stream, DOWN-pointing chevrons moving DOWN"""
        x = self.screen_width / 2
        count = int(self.screen_height / self.chevron_spacing) + 3
        
        for i in range(-1, count):
            y = i * self.chevron_spacing + self.offset
            
            if -self.chevron_spacing <= y <= self.screen_height + self.chevron_spacing:
                points = [
                    (x, y + self.chevron_height / 2),
                    (x + self.chevron_width / 2, y - self.chevron_height / 2),
                    (x - self.chevron_width / 2, y - self.chevron_height / 2),
                ]
                pygame.draw.polygon(self.screen, (255, 255, 255), points)
    
    def draw_up_chevrons(self):
        """UP - horizontal stream, LEFT-pointing chevrons moving LEFT"""
        y = self.screen_height / 2
        count = int(self.screen_width / self.chevron_spacing) + 3
        
        for i in range(-1, count):
            x = i * self.chevron_spacing - self.offset
            
            if -self.chevron_spacing <= x <= self.screen_width + self.chevron_spacing:
                points = [
                    (x + self.chevron_width / 2, y - self.chevron_height / 2),
                    (x - self.chevron_width / 2, y),
                    (x + self.chevron_width / 2, y + self.chevron_height / 2),
                ]
                pygame.draw.polygon(self.screen, (255, 255, 255), points)
    
    def draw_down_chevrons(self):
        """DOWN - horizontal stream, RIGHT-pointing chevrons moving RIGHT"""
        y = self.screen_height / 2
        count = int(self.screen_width / self.chevron_spacing) + 3
        
        for i in range(-1, count):
            x = i * self.chevron_spacing + self.offset
            
            if -self.chevron_spacing <= x <= self.screen_width + self.chevron_spacing:
                points = [
                    (x - self.chevron_width / 2, y - self.chevron_height / 2),
                    (x + self.chevron_width / 2, y),
                    (x - self.chevron_width / 2, y + self.chevron_height / 2),
                ]
                pygame.draw.polygon(self.screen, (255, 255, 255), points)
    
    def destroy_node(self):
        """Cleanup"""
        pygame.quit()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ChevronCmdVelNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
