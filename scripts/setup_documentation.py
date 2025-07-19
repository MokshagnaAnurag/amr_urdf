#!/usr/bin/env python3
"""
Complete documentation setup script for AMR project
Sets up all visualization assets and documentation images
"""

import os
import sys
import subprocess
import importlib

def check_dependencies():
    """Check if required dependencies are installed"""
    required_packages = {
        'numpy': 'numpy',
        'matplotlib': 'matplotlib', 
        'PIL': 'Pillow',
        'trimesh': 'trimesh'
    }
    
    missing_packages = []
    available_packages = []
    
    print("🔍 Checking dependencies...")
    
    for module, package in required_packages.items():
        try:
            importlib.import_module(module)
            available_packages.append(f"✅ {package}")
        except ImportError:
            missing_packages.append(package)
            available_packages.append(f"❌ {package}")
    
    # Print status
    for status in available_packages:
        print(f"   {status}")
    
    if missing_packages:
        print(f"\n⚠️  Missing packages: {', '.join(missing_packages)}")
        print("📦 Install with: pip install " + " ".join(missing_packages))
        return False
    else:
        print("✅ All dependencies available!")
        return True

def setup_directories():
    """Create necessary directories"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)
    
    directories = {
        'images': os.path.join(project_root, 'images'),
        'docs': os.path.join(project_root, 'docs'),
        'assets': os.path.join(project_root, 'assets')
    }
    
    print("\n📁 Setting up directories...")
    for name, path in directories.items():
        os.makedirs(path, exist_ok=True)
        print(f"   ✅ {name}: {path}")
    
    return directories

def create_placeholder_images(images_dir):
    """Create placeholder images if STL conversion fails"""
    try:
        import matplotlib.pyplot as plt
        import matplotlib.patches as patches
        
        components = ['base_link', 'LW_1', 'LW_2', 'RW_1', 'RW_2', 'Lidar']
        colors = {
            'base_link': '#A0A0A4',
            'LW_1': '#4A90E2', 
            'LW_2': '#4A90E2',
            'RW_1': '#F5F5F5',
            'RW_2': '#F5F5F5',
            'Lidar': '#2C3E50'
        }
        
        print("\n🎨 Creating placeholder images...")
        
        for component in components:
            fig, ax = plt.subplots(1, 1, figsize=(8, 6), dpi=150)
            
            # Create simple geometric representation
            if 'base_link' in component:
                shape = patches.Rectangle((0.2, 0.3), 0.6, 0.4, 
                                        facecolor=colors[component], alpha=0.8)
            elif 'Lidar' in component:
                shape = patches.Circle((0.5, 0.5), 0.15, 
                                     facecolor=colors[component], alpha=0.8)
            else:  # wheels
                shape = patches.Circle((0.5, 0.5), 0.2, 
                                     facecolor=colors[component], alpha=0.8)
            
            ax.add_patch(shape)
            ax.set_xlim(0, 1)
            ax.set_ylim(0, 1)
            ax.set_aspect('equal')
            ax.axis('off')
            
            # Add title
            title = component.replace('_', ' ').title()
            ax.set_title(f'{title}\n(Placeholder - Run STL converter for actual model)', 
                        fontsize=12, fontweight='bold', pad=20)
            
            # Save image
            output_path = os.path.join(images_dir, f'{component}.png')
            plt.savefig(output_path, dpi=150, bbox_inches='tight', 
                       facecolor='white', edgecolor='none')
            plt.close()
            
            print(f"   📷 {component}.png")
        
        return True
        
    except Exception as e:
        print(f"❌ Error creating placeholder images: {e}")
        return False

def run_stl_conversion():
    """Run the STL to images conversion"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    stl_script = os.path.join(script_dir, 'stl_to_images.py')
    
    print("\n🚀 Running STL conversion...")
    
    try:
        # Check if STL files exist
        meshes_dir = os.path.join(os.path.dirname(script_dir), 'meshes')
        stl_files = [f for f in os.listdir(meshes_dir) if f.endswith('.STL')]
        
        if not stl_files:
            print("⚠️  No STL files found in meshes directory")
            return False
        
        print(f"📁 Found {len(stl_files)} STL files: {', '.join(stl_files)}")
        
        # Run conversion script
        result = subprocess.run([sys.executable, stl_script], 
                              capture_output=True, text=True, cwd=script_dir)
        
        if result.returncode == 0:
            print("✅ STL conversion completed successfully!")
            return True
        else:
            print(f"❌ STL conversion failed: {result.stderr}")
            return False
            
    except Exception as e:
        print(f"❌ Error running STL conversion: {e}")
        return False

def create_banner():
    """Create banner image"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    banner_script = os.path.join(script_dir, 'create_banner.py')
    
    print("\n🎨 Creating banner image...")
    
    try:
        result = subprocess.run([sys.executable, banner_script], 
                              capture_output=True, text=True, cwd=script_dir)
        
        if result.returncode == 0:
            print("✅ Banner created successfully!")
            return True
        else:
            print(f"❌ Banner creation failed: {result.stderr}")
            return False
            
    except Exception as e:
        print(f"❌ Error creating banner: {e}")
        return False

def generate_documentation_assets():
    """Generate all documentation assets"""
    print("🤖 AMR Documentation Setup")
    print("=" * 50)
    
    # Check dependencies
    deps_ok = check_dependencies()
    
    # Setup directories
    directories = setup_directories()
    
    # Create banner
    banner_ok = create_banner()
    
    # Try STL conversion
    stl_ok = False
    if deps_ok:
        stl_ok = run_stl_conversion()
    
    # Create placeholders if STL conversion failed
    if not stl_ok:
        print("\n📷 STL conversion failed or unavailable, creating placeholders...")
        placeholder_ok = create_placeholder_images(directories['images'])
    
    # Summary
    print("\n📊 Setup Summary:")
    print(f"   📁 Directories: ✅")
    print(f"   🎨 Banner: {'✅' if banner_ok else '❌'}")
    print(f"   🔧 STL Conversion: {'✅' if stl_ok else '❌'}")
    print(f"   📷 Images: {'✅' if stl_ok or placeholder_ok else '❌'}")
    
    if not deps_ok:
        print(f"\n💡 To enable full functionality, install missing dependencies:")
        print(f"   pip install numpy matplotlib Pillow trimesh")
    
    print(f"\n🎉 Documentation setup complete!")
    print(f"📁 Images saved to: {directories['images']}")

if __name__ == "__main__":
    generate_documentation_assets()