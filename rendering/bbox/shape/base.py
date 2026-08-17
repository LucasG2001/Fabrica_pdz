import math

import bpy # pylint: disable=import-error
import numpy as np

from ..base import BaseObj
from ..mat import BaseMat
from ..utils import Vec3, Mat44


class BaseShape(BaseObj):

    def __init__(self, bpy_obj: bpy.types.Object) -> None:
        super().__init__(bpy_obj)
        self.set_normal_mode('face')
        self.update()

    def set_location(self, location: Vec3) -> None:
        self.bpy_obj.location = location

    def set_rotation(self, rotation: Vec3) -> None:
        self.bpy_obj.rotation_euler = rotation

    def set_scale(self, scale: Vec3 | int | float) -> None:
        if isinstance(scale, (int, float)):
            scale = np.full(3, scale)
        self.bpy_obj.scale = scale

    def set_transformation(self, transformation: Mat44) -> None:
        self.bpy_obj.matrix_world = transformation

    def update(self) -> None:
        bpy.context.view_layer.update()

    def invert_normals(self) -> None:
        self.bpy_data.flip_normals()

    def subdivide(self, level: int = 0) -> None:
        bpy_obj = self.bpy_obj
        bpy.context.view_layer.objects.active = bpy_obj
        bpy.ops.object.modifier_add(type='SUBSURF')
        bpy_obj.modifiers["Subdivision"].render_levels = level
        bpy_obj.modifiers['Subdivision'].levels = level

    def set_shadow(self, mode: bool = True) -> None:
        self.bpy_obj.is_shadow_catcher = mode # for blender 3.X

    def set_normal_mode(self, mode: str = 'vertex', **kwargs) -> None:
        bpy_mesh = self.bpy_data
        if mode.casefold() == 'vertex':
            # Blender >= 4.1: auto smooth moved from Mesh props to the shade_smooth_by_angle operator
            kwargs.pop('use_auto_smooth', None)
            angle = math.radians(kwargs.pop('auto_smooth_angle', 30.0))
            bpy_obj = self.bpy_obj
            bpy.context.view_layer.objects.active = bpy_obj
            bpy_obj.select_set(True)
            bpy.ops.object.shade_smooth_by_angle(angle=angle)
        elif mode.casefold() == 'face':
            bpy_mesh.shade_flat()
        else:
            raise ValueError(f'unsupported normal mode: {mode}')

    def set_mat(self, mat: BaseMat) -> None:
        bpy_obj = self.bpy_obj
        bpy_mat = mat.bpy_data
        bpy_obj.data.materials.append(bpy_mat)
        bpy_obj.active_material = bpy_mat

    @property
    def num_verts(self) -> int:
        return len(self.bpy_data.vertices)

    @property
    def num_faces(self) -> int:
        return len(self.bpy_data.polygons)
