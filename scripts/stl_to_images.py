#!/usr/bin/env python3
"""
STL to Image Converter Script for AMR Project
Converts STL files from Fusion 360/SolidWorks to PNG, JPEG, and GIF formats
Compatible with professional CAD workflows
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import warnings
warnings.filterwarnings("ignore")

try:
    import trimesh
    TRIMESH_AVAILABLE = True
except ImportError:
    TRIMESH_AVAILABLE = False
    print("Warning: trimesh not available. Install with: pip install trimesh")

try:
    from PIL import Image, ImageDraw, ImageFont
    PIL_AVAILABLE = True
except ImportError:
    PIL_AVAILABLE = False
    print("Warning: PIL not available. Install with: pip install Pillow")

class STLConverter:
    def __init__(self, meshes_dir, output_dir):
        self.meshes_dir = meshes_dir
        self.output_dir = output_dir
        
        # Create output directory if it doesn't exist
        os.makedirs(output_dir, exist_ok=True)
        
        # Professional color scheme matching CAD software appearance
        self.colors = {
            'base_link': '#A0A0A4',      # Aluminum gray
            'LW_1': '#4A90E2',           # Professional blue
            'LW_2': '#4A90E2',           # Professional blue  
            'RW_1': '#F5F5F5',           # Off-white
            'RW_2': '#F5F5F5',           # Off-white
            'Lidar': '#2C3E50'           # Dark blue-gray
        }
        
        # Component descriptions for professional documentation
        self.descriptions = {
            'base_link': 'Main Chassis - Primary structural component',
            'LW_1': 'Left Wheel 1 - Front left drive wheel',
            'LW_2': 'Left Wheel 2 - Rear left drive wheel',
            'RW_1': 'Right Wheel 1 - Front right drive wheel', 
            'RW_2': 'Right Wheel 2 - Rear right drive wheel',
            'Lidar': 'LiDAR Sensor - 360° laser range finder'
        }
    
    def read_stl_simple(self, filename):
        """Simple STL reader for basic triangle mesh data"""
        try:
            vertices = []
            faces = []
            
            with open(filename, 'rb') as file:
                # Skip header (80 bytes)
                file.read(80)
                
                # Read number of triangles
                num_triangles = int.from_bytes(file.read(4), byteorder='little')
                
                vertex_dict = {}
                vertex_index = 0
                
                for i in range(num_triangles):
                    # Skip normal vector (12 bytes)
                    file.read(12)
                    
                    # Read 3 vertices (36 bytes total)
                    triangle_vertices = []
                    for j in range(3):
                        x = np.frombuffer(file.read(4), dtype=np.float32)[0]
                        y = np.frombuffer(file.read(4), dtype=np.float32)[0] 
                        z = np.frombuffer(file.read(4), dtype=np.float32)[0]
                        
                        vertex = (x, y, z)
                        if vertex not in vertex_dict:
                            vertex_dict[vertex] = vertex_index
                            vertices.append([x, y, z])
                            vertex_index += 1
                        
                        triangle_vertices.append(vertex_dict[vertex])
                    
                    faces.append(triangle_vertices)
                    
                    # Skip attribute byte count (2 bytes)
                    file.read(2)
            
            return np.array(vertices), np.array(faces)
            
        except Exception as e:
            print(f"Error reading STL file {filename}: {e}")
            return None, None
    
    def load_stl(self, filename):
        """Load STL file with fallback methods"""
        if TRIMESH_AVAILABLE:
            try:
                mesh = trimesh.load_mesh(filename)
                return mesh.vertices, mesh.faces
            except Exception as e:
                print(f"Trimesh failed, using simple reader: {e}")
        
        # Fallback to simple reader
        return self.read_stl_simple(filename)
    
    def create_professional_static_image(self, vertices, faces, filename, title, format='png'):
        """Create professional-quality static image from mesh"""
        fig = plt.figure(figsize=(12, 10), dpi=300)
        ax = fig.add_subplot(111, projection='3d')
        
        # Create 3D collection
        mesh_collection = Poly3DCollection(vertices[faces], alpha=0.85)
        
        # Set professional styling
        component_name = os.path.splitext(os.path.basename(filename))[0]
        color = self.colors.get(component_name, '#A0A0A4')
        mesh_collection.set_facecolor(color)
        mesh_collection.set_edgecolor('#2C3E50')
        mesh_collection.set_linewidth(0.15)
        
        ax.add_collection3d(mesh_collection)
        
        # Calculate optimal view bounds
        max_range = np.array([
            vertices[:, 0].max() - vertices[:, 0].min(),
            vertices[:, 1].max() - vertices[:, 1].min(),
            vertices[:, 2].max() - vertices[:, 2].min()
        ]).max() / 2.0
        
        mid_x = (vertices[:, 0].max() + vertices[:, 0].min()) * 0.5
        mid_y = (vertices[:, 1].max() + vertices[:, 1].min()) * 0.5
        mid_z = (vertices[:, 2].max() + vertices[:, 2].min()) * 0.5
        
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)
        
        # Professional styling
        ax.set_title(f'{title}', fontsize=18, fontweight='bold', pad=20)
        
        # Add component description
        description = self.descriptions.get(component_name, '')
        if description:
            ax.text2D(0.5, 0.02, description, transform=ax.transAxes, 
                     ha='center', fontsize=12, style='italic')
        
        ax.set_xlabel('X (mm)', fontsize=14, labelpad=10)
        ax.set_ylabel('Y (mm)', fontsize=14, labelpad=10)
        ax.set_zlabel('Z (mm)', fontsize=14, labelpad=10)
        
        # Professional viewing angle
        ax.view_init(elev=25, azim=-60)
        
        # Clean background
        ax.xaxis.pane.fill = False
        ax.yaxis.pane.fill = False
        ax.zaxis.pane.fill = False
        ax.grid(True, alpha=0.3)
        
        # Set background color
        ax.xaxis.pane.set_edgecolor('gray')
        ax.yaxis.pane.set_edgecolor('gray')
        ax.zaxis.pane.set_edgecolor('gray')
        
        # Save with high quality
        output_filename = os.path.join(self.output_dir, f'{component_name}.{format}')
        plt.savefig(output_filename, format=format, dpi=300, bbox_inches='tight', 
                   facecolor='white', edgecolor='none', pad_inches=0.2)
        plt.close()
        
        print(f"✅ Created: {output_filename}")
        return output_filename
    
    def create_professional_gif(self, vertices, faces, filename, title, num_frames=72):
        """Create smooth rotating GIF animation"""
        component_name = os.path.splitext(os.path.basename(filename))[0]
        
        if not PIL_AVAILABLE:
            print("❌ PIL not available, skipping GIF creation")
            return None
        
        # Create temporary frames
        frame_files = []
        
        print(f"🎬 Creating {num_frames} animation frames...")
        
        for i in range(num_frames):
            fig = plt.figure(figsize=(10, 10), dpi=150)
            ax = fig.add_subplot(111, projection='3d')
            
            # Create 3D collection
            mesh_collection = Poly3DCollection(vertices[faces], alpha=0.9)
            
            # Set professional styling
            color = self.colors.get(component_name, '#A0A0A4')
            mesh_collection.set_facecolor(color)
            mesh_collection.set_edgecolor('#2C3E50')
            mesh_collection.set_linewidth(0.1)
            
            ax.add_collection3d(mesh_collection)
            
            # Set equal aspect ratio
            max_range = np.array([
                vertices[:, 0].max() - vertices[:, 0].min(),
                vertices[:, 1].max() - vertices[:, 1].min(),
                vertices[:, 2].max() - vertices[:, 2].min()
            ]).max() / 2.0
            
            mid_x = (vertices[:, 0].max() + vertices[:, 0].min()) * 0.5
            mid_y = (vertices[:, 1].max() + vertices[:, 1].min()) * 0.5
            mid_z = (vertices[:, 2].max() + vertices[:, 2].min()) * 0.5
            
            ax.set_xlim(mid_x - max_range, mid_x + max_range)
            ax.set_ylim(mid_y - max_range, mid_y + max_range)
            ax.set_zlim(mid_z - max_range, mid_z + max_range)
            
            # Clean look for animation
            ax.set_axis_off()
            
            # Smooth rotation
            angle = i * (360 / num_frames)
            ax.view_init(elev=20, azim=angle)
            
            # Save frame
            frame_file = os.path.join(self.output_dir, f'temp_frame_{component_name}_{i:03d}.png')
            plt.savefig(frame_file, format='png', dpi=150, bbox_inches='tight',
                       facecolor='white', edgecolor='none', pad_inches=0)
            plt.close()
            
            frame_files.append(frame_file)
            
            # Progress indicator
            if (i + 1) % 10 == 0:
                print(f"   📷 Frame {i + 1}/{num_frames}")
        
        # Create GIF from frames
        frames = []
        for frame_file in frame_files:
            frames.append(Image.open(frame_file))
        
        # Save as optimized GIF
        gif_filename = os.path.join(self.output_dir, f'{component_name}.gif')
        frames[0].save(gif_filename, save_all=True, append_images=frames[1:],
                      duration=80, loop=0, optimize=True)
        
        # Clean up temporary frames
        for frame_file in frame_files:
            try:
                os.remove(frame_file)
            except:
                pass
        
        print(f"✅ Created: {gif_filename}")
        return gif_filename
    
    def create_assembly_overview(self):
        """Create an overview image showing all components"""
        print("🔧 Creating assembly overview...")
        
        fig = plt.figure(figsize=(16, 12), dpi=300)
        
        stl_files = [f for f in os.listdir(self.meshes_dir) if f.endswith('.STL')]
        num_components = len(stl_files)
        
        if num_components == 0:
            print("❌ No STL files found")
            return
        
        # Calculate grid layout
        cols = 3
        rows = (num_components + cols - 1) // cols
        
        for i, stl_file in enumerate(stl_files):
            stl_path = os.path.join(self.meshes_dir, stl_file)
            vertices, faces = self.load_stl(stl_path)
            
            if vertices is None:
                continue
            
            ax = fig.add_subplot(rows, cols, i + 1, projection='3d')
            
            # Create mesh collection
            mesh_collection = Poly3DCollection(vertices[faces], alpha=0.8)
            
            component_name = os.path.splitext(stl_file)[0]
            color = self.colors.get(component_name, '#A0A0A4')
            mesh_collection.set_facecolor(color)
            mesh_collection.set_edgecolor('#2C3E50')
            mesh_collection.set_linewidth(0.1)
            
            ax.add_collection3d(mesh_collection)
            
            # Set bounds
            max_range = np.array([
                vertices[:, 0].max() - vertices[:, 0].min(),
                vertices[:, 1].max() - vertices[:, 1].min(),
                vertices[:, 2].max() - vertices[:, 2].min()
            ]).max() / 2.0
            
            mid_x = (vertices[:, 0].max() + vertices[:, 0].min()) * 0.5
            mid_y = (vertices[:, 1].max() + vertices[:, 1].min()) * 0.5
            mid_z = (vertices[:, 2].max() + vertices[:, 2].min()) * 0.5
            
            ax.set_xlim(mid_x - max_range, mid_x + max_range)
            ax.set_ylim(mid_y - max_range, mid_y + max_range)
            ax.set_zlim(mid_z - max_range, mid_z + max_range)
            
            # Style
            ax.set_title(component_name.replace('_', ' ').title(), fontsize=12, fontweight='bold')
            ax.set_axis_off()
            ax.view_init(elev=20, azim=-60)
        
        plt.suptitle('AMR Components Overview', fontsize=20, fontweight='bold', y=0.95)
        plt.tight_layout()
        
        overview_file = os.path.join(self.output_dir, 'complete_robot.png')
        plt.savefig(overview_file, format='png', dpi=300, bbox_inches='tight',
                   facecolor='white', edgecolor='none')
        plt.close()
        
        print(f"✅ Created: {overview_file}")
    
    def convert_all_stl_files(self):
        """Convert all STL files in the meshes directory"""
        stl_files = [f for f in os.listdir(self.meshes_dir) if f.endswith('.STL')]
        
        if not stl_files:
            print("❌ No STL files found in meshes directory")
            return
        
        print(f"🚀 Found {len(stl_files)} STL files from CAD software")
        print("📁 Files to process:", ', '.join(stl_files))
        
        # Create assembly overview first
        self.create_assembly_overview()
        
        for i, stl_file in enumerate(stl_files, 1):
            print(f"\n🔄 Processing ({i}/{len(stl_files)}): {stl_file}")
            
            stl_path = os.path.join(self.meshes_dir, stl_file)
            vertices, faces = self.load_stl(stl_path)
            
            if vertices is None:
                print(f"❌ Failed to load: {stl_file}")
                continue
            
            # Generate professional title
            component_name = os.path.splitext(stl_file)[0]
            title = component_name.replace('_', ' ').title()
            
            try:
                # Create static images
                self.create_professional_static_image(vertices, faces, stl_path, title, 'png')
                self.create_professional_static_image(vertices, faces, stl_path, title, 'jpg')
                
                # Create rotating GIF
                self.create_professional_gif(vertices, faces, stl_path, title)
                
            except Exception as e:
                print(f"❌ Error processing {stl_file}: {e}")
        
        print(f"\n🎉 Conversion complete! Professional images saved to: {self.output_dir}")
        print("📊 Generated files:")
        print("   • PNG images (high-resolution static views)")
        print("   • JPEG images (documentation format)")  
        print("   • GIF animations (360° rotating views)")
        print("   • Assembly overview (all components)")

def main():
    print("🤖 AMR STL to Images Converter")
    print("🔧 Optimized for Fusion 360 & SolidWorks STL files")
    print("=" * 60)
    
    # Setup paths
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)
    meshes_dir = os.path.join(project_root, 'meshes')
    output_dir = os.path.join(project_root, 'images')
    
    print(f"📂 Meshes directory: {meshes_dir}")
    print(f"📂 Output directory: {output_dir}")
    
    # Check if meshes directory exists
    if not os.path.exists(meshes_dir):
        print(f"❌ Error: Meshes directory not found: {meshes_dir}")
        print("💡 Make sure your STL files from Fusion 360/SolidWorks are in the meshes folder")
        sys.exit(1)
    
    # Check for dependencies
    print("\n🔍 Checking dependencies...")
    if TRIMESH_AVAILABLE:
        print("✅ trimesh available")
    else:
        print("⚠️  trimesh not available (using fallback STL reader)")
        
    if PIL_AVAILABLE:
        print("✅ PIL available")
    else:
        print("⚠️  PIL not available (GIF creation disabled)")
    
    # Create converter and process files
    converter = STLConverter(meshes_dir, output_dir)
    converter.convert_all_stl_files()

if __name__ == "__main__":
    main()