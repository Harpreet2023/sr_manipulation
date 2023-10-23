# Software License Agreement (BSD License)
# Copyright (c) 2012, Willow Garage, Inc.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of Willow Garage nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
# Author: Ioan Sucan, Sachin Chitta
# based on planning_scene_interface.cpp


# part of make_mesh copied from planning_scene_interface.py of moveit_commander
# original authors: Ioan Sucan, Felix Messmer, same License as above
try:
    import pyassimp
    from pyassimp import load

except:
    pyassimp = False
    print("Failed to import pyassimp")

from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle


# make_mesh copied from planning_scene_interface.py of moveit_commander
# original authors: Ioan Sucan, Felix Messmer, same License as above
def make_mesh(node:Node, uri: str, scale = (1, 1, 1)):
    if pyassimp is False:
        node.get_logger().warn("Pyassimp not found")
        return None
    try:
        # TODO(gwalck) handle uri in a cleaner way, maybe even with packages
        if uri.startswith("file://"):
            filename = uri[7:len(uri)]
        else:
            filename = uri
        with load(filename) as scene:
            if not scene.meshes or len(scene.meshes) == 0:
                node.get_logger().warn("There are no meshes in the file")
                return None
            if len(scene.meshes[0].faces) == 0:
                node.get_logger().warn("There are no faces in the mesh")
                return None

            mesh = Mesh()
            first_face = scene.meshes[0].faces[0]
            if hasattr(first_face, '__len__'):
                for face in scene.meshes[0].faces:
                    if len(face) == 3:
                        triangle = MeshTriangle()
                        triangle.vertex_indices = numpy.array([face[0], face[1], face[2]], dtype=numpy.uint32)
                        mesh.triangles.append(triangle)
            elif hasattr(first_face, 'indices'):
                for face in scene.meshes[0].faces:
                    if len(face.indices) == 3:
                        triangle = MeshTriangle()
                        triangle.vertex_indices = numpy.array([face.indices[0],
                                                face.indices[1],
                                                face.indices[2]],dtype=numpy.uint32)
                        mesh.triangles.append(triangle)
            else:
                node.get_logger().warn("Unable to build triangles from mesh due to mesh object structure")
                return None
            for vertex in scene.meshes[0].vertices:
                point = Point()
                point.x = vertex[0]*scale[0]
                point.y = vertex[1]*scale[1]
                point.z = vertex[2]*scale[2]
                mesh.vertices.append(point)
    except Exception as e:
        node.get_logger().error(f'Failed to load mesh file {filename}, {e}')
        return None
    return mesh
