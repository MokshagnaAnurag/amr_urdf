#!/usr/bin/env python3
"""
Simple banner image creator for AMR README
Creates a professional banner image using matplotlib
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import os

def create_banner():
    """Create a professional banner image"""
    
    # Create figure
    fig, ax = plt.subplots(1, 1, figsize=(12, 4), dpi=300)
    
    # Set background gradient
    gradient = np.linspace(0, 1, 256).reshape(1, -1)
    gradient = np.vstack((gradient, gradient))
    
    # Create background
    ax.imshow(gradient, extent=[0, 12, 0, 4], aspect='auto', cmap='Blues_r', alpha=0.3)
    
    # Add title
    ax.text(6, 2.5, 'AMR', fontsize=48, fontweight='bold', 
            ha='center', va='center', color='#2C3E50')
    
    ax.text(6, 1.8, 'Autonomous Mobile Robot', fontsize=20, 
            ha='center', va='center', color='#34495E')
    
    ax.text(6, 1.4, 'Professional Design • Fusion 360 • SolidWorks • ROS2', 
            fontsize=12, ha='center', va='center', color='#7F8C8D', style='italic')
    
    # Add robot-like elements
    # Main body
    robot_body = patches.Rectangle((1.5, 1.2), 1.5, 1.0, 
                                  linewidth=2, edgecolor='#2C3E50', 
                                  facecolor='#A0A0A4', alpha=0.8)
    ax.add_patch(robot_body)
    
    # Wheels
    wheel1 = patches.Circle((1.3, 1.2), 0.3, linewidth=2, 
                           edgecolor='#2C3E50', facecolor='#4A90E2', alpha=0.8)
    wheel2 = patches.Circle((3.2, 1.2), 0.3, linewidth=2, 
                           edgecolor='#2C3E50', facecolor='#4A90E2', alpha=0.8)
    wheel3 = patches.Circle((1.3, 2.2), 0.3, linewidth=2, 
                           edgecolor='#2C3E50', facecolor='#F5F5F5', alpha=0.8)
    wheel4 = patches.Circle((3.2, 2.2), 0.3, linewidth=2, 
                           edgecolor='#2C3E50', facecolor='#F5F5F5', alpha=0.8)
    
    ax.add_patch(wheel1)
    ax.add_patch(wheel2)
    ax.add_patch(wheel3)
    ax.add_patch(wheel4)
    
    # LiDAR
    lidar = patches.Circle((2.25, 2.5), 0.2, linewidth=2, 
                          edgecolor='#2C3E50', facecolor='#2C3E50', alpha=0.8)
    ax.add_patch(lidar)
    
    # Add scanning lines from LiDAR
    angles = np.linspace(0, 2*np.pi, 8)
    for angle in angles:
        x_end = 2.25 + 0.5 * np.cos(angle)
        y_end = 2.5 + 0.5 * np.sin(angle)
        ax.plot([2.25, x_end], [2.5, y_end], 'r-', alpha=0.6, linewidth=1)
    
    # Mirror robot on the right side
    robot_body2 = patches.Rectangle((9, 1.2), 1.5, 1.0, 
                                   linewidth=2, edgecolor='#2C3E50', 
                                   facecolor='#A0A0A4', alpha=0.8)
    ax.add_patch(robot_body2)
    
    # Technical elements
    ax.text(0.5, 3.5, '🔧', fontsize=24, ha='center', va='center')
    ax.text(11.5, 3.5, '⚙️', fontsize=24, ha='center', va='center')
    ax.text(0.5, 0.5, '📐', fontsize=24, ha='center', va='center')
    ax.text(11.5, 0.5, '🤖', fontsize=24, ha='center', va='center')
    
    # Remove axes
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 4)
    ax.set_aspect('equal')
    ax.axis('off')
    
    # Save banner
    script_dir = os.path.dirname(os.path.abspath(__file__))
    images_dir = os.path.join(os.path.dirname(script_dir), 'images')
    os.makedirs(images_dir, exist_ok=True)
    
    banner_path = os.path.join(images_dir, 'banner.png')
    plt.savefig(banner_path, dpi=300, bbox_inches='tight', 
                facecolor='white', edgecolor='none', pad_inches=0.1)
    plt.close()
    
    print(f"✅ Banner created: {banner_path}")
    return banner_path

if __name__ == "__main__":
    create_banner()