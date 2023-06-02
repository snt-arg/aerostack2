"""
object.py
"""

# Copyright 2022 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


__authors__ = "Pedro Arias Pérez"
__copyright__ = "Copyright (c) 2022 Universidad Politécnica de Madrid"
__license__ = "BSD-3-Clause"
__version__ = "0.1.0"


import os
from enum import Enum
from ign_assets.bridges.bridge import Bridge
from ign_assets.bridges import bridges as ign_bridges
from ign_assets.bridges import custom_bridges as ign_custom_bridges
from ign_assets.models.entity import Entity
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


class ObjectBridgesTypeEnum(str, Enum):
    """Valid drone model types"""
    GPS = 'gps'
    AZIMUTH = 'azimuth'
    POSE = 'pose'

    def bridges(self, world_name: str, model_name: str) -> tuple[list[Bridge], list[Node]]:
        """Return associated bridge or custom bridge to BridgeType.
        First list of bridges, the list of nodes.
        """
        if self.name == self.GPS.name:
            return [], [ign_custom_bridges.gps_node(
                world_name, model_name, 'gps', 'gps')]
        if self.name == self.AZIMUTH.name:
            return [], [ign_custom_bridges.azimuth_node(model_name)]
        if self.name == self.POSE.name:
            return [ign_bridges.pose(model_name)], []
        return [], []


class Object(Entity):
    """Gz Object Entity"""
    joints: list[str] = []
    object_bridges: list[ObjectBridgesTypeEnum] = []
    tf_broadcaster: bool = True
    use_sim_time: bool = True

    def bridges(self, world_name: str):
        """Object bridges
        """
        bridges = self.joint_bridges()
        nodes = []
        if self.tf_broadcaster:
            nodes.append(ign_custom_bridges.tf_broadcaster_node(
                world_name, self.model_name, 'earth', self.use_sim_time))

        for bridge in self.object_bridges:
            bridges_, nodes_ = bridge.bridges(world_name, self.model_name)
            bridges.extend(bridges_)
            nodes.extend(nodes_)
        return bridges, nodes

    def joint_bridges(self) -> list[Bridge]:
        """Return gz_to_ros bridges needed for the object to move"""
        bridges = []
        for joint in self.joints:
            bridges.append(ign_bridges.joint_cmd_vel(
                self.model_name, joint))
        return bridges

    def generate(self, world) -> tuple[str, str]:
        """Object are not jinja templates, no need for creating, using base one"""
        model_dir = os.path.join(
            get_package_share_directory('as2_ign_gazebo_assets'), 'models')

        model_sdf = f'{model_dir}/{self.model_type}/{self.model_type}.sdf'
        return "", model_sdf
