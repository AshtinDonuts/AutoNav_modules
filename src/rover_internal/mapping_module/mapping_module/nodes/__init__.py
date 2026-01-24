"""
Nodes package for mapping_module

This package contains ROS2 node implementations.
"""

# Lazy imports to avoid loading heavy dependencies (cv2, torch, etc.) 
# when only specific nodes are needed. Import directly from node files instead.
# Example: from mapping_module.nodes.zed_3dpc_node import ZedPointCloudNode

__all__ = []
