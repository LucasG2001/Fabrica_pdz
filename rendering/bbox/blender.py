from pathlib import Path

import bpy # pylint: disable=import-error

from .cam import BaseCam
from .utils import PathLike, Color


class Blender(object):
    def __init__(
            self,
            samples: int = 256,
            max_bounces: int = 8,
            film_exposure: float = 1.0,
            width: int = 1024,
            height: int = 1024,
            resolution_percentage: int = 100,
            use_denoise: bool = False,
            use_gpu: bool = True,
            engine: str = 'CYCLES', # 'CYCLES' or 'BLENDER_EEVEE'
            transparent: bool = True) -> None:

        self.reset()

        scene = self.scene

        scene.render.engine = engine # set render engine

        scene.cursor.location = (0, 0, 0) # set cursor location to (0, 0, 0)
        scene.cycles.samples = samples # set samples
        scene.cycles.max_bounces = max_bounces # set max_bounces
        scene.cycles.film_exposure = film_exposure # set film_exposure
        scene.render.film_transparent = transparent # set transparent background
        scene.cycles.use_denoising = use_denoise # set use_denoising
        scene.view_layers[0]['cycles']['use_denoising'] = 1 if use_denoise else 0 # set use_denoising

        # set devices
        cycle_pref = bpy.context.preferences.addons['cycles'].preferences
        if use_gpu:
            scene.cycles.device = 'GPU'
            for s in bpy.data.scenes:
                s.cycles.device = 'GPU'
            cycle_pref.compute_device_type = 'CUDA'
            for d in cycle_pref.get_devices_for_type('CUDA'):
                d.use = True
        else:
            scene.cycles.device = 'CPU'
        for dev in cycle_pref.devices:
            print(f'found device: {dev.name}, type: {dev.type}, use: {dev.use}')

        self.width = width
        self.height = height
        self.resolution_percentage = resolution_percentage
        self.transparent = transparent

    def reset(self) -> None:
        bpy.ops.wm.read_homefile() # reset the scene
        bpy.ops.object.select_all(action='SELECT') # select all objects
        bpy.ops.object.delete(use_global=False) # delete all objects
        # bpy.data.meshes.remove(bpy.data.meshes['Cube']) # remove the default cube
        # bpy.data.materials.remove(bpy.data.materials['Material']) # remove the default material
        # bpy.data.collections.remove(bpy.data.collections['Collection']) # remove the default collection

    @property
    def scene(self) -> bpy.types.Scene:
        return bpy.context.scene

    def set_ambient_light(self, color: Color) -> None:
        world = self.scene.world
        world.use_nodes = True
        world.node_tree.nodes['Background'].inputs['Color'].default_value = color.rgba

    # set gray shadow to completely white with a threshold (optional but recommended)
    def set_shadow_threshold(self, threshold: float) -> None:
        scene = self.scene
        scene.use_nodes = True
        tree = scene.compositing_node_group
        if tree is None:
            # Blender >= 4.0: the compositor is a node group, not an implicit scene.node_tree
            tree = bpy.data.node_groups.new('Compositor', 'CompositorNodeTree')
            tree.interface.new_socket(name='Image', in_out='OUTPUT', socket_type='NodeSocketColor')
            tree.interface.new_socket(name='Alpha', in_out='OUTPUT', socket_type='NodeSocketFloat')
            scene.compositing_node_group = tree

        render_layers = tree.nodes.get('Render Layers') or tree.nodes.new('CompositorNodeRLayers')
        render_layers.name = 'Render Layers'
        composite = tree.nodes.get('Group Output') or tree.nodes.new('NodeGroupOutput')
        tree.links.new(render_layers.outputs['Image'], composite.inputs['Image'])

        # ColorRamp ('ValToRGB') is now a generic shader node usable inside the compositor tree
        compositor_node_val_to_rgb = tree.nodes.new('ShaderNodeValToRGB')
        compositor_node_val_to_rgb.color_ramp.elements[0].color[3] = 0
        compositor_node_val_to_rgb.color_ramp.elements[0].position = threshold
        compositor_node_val_to_rgb.color_ramp.interpolation = 'CARDINAL'

        tree.links.new(render_layers.outputs['Alpha'], compositor_node_val_to_rgb.inputs[0])
        tree.links.new(compositor_node_val_to_rgb.outputs['Alpha'], composite.inputs['Alpha'])

    def save(self, path: PathLike) -> None:
        path = Path(path).resolve()
        bpy.ops.wm.save_as_mainfile(filepath=str(path))

    def set_background(self, color: Color) -> None:
        world = self.scene.world
        world.use_nodes = True
        world.node_tree.nodes['Background'].inputs['Color'].default_value = color.rgba

    def render(self, path: PathLike, cam: BaseCam) -> None:

        path = Path(path).resolve()

        scene = self.scene
        scene.camera = cam.bpy_obj # set camera
        scene.render.filepath = str(path) # set filepath
        scene.render.engine = 'CYCLES' # set render engine to cycles
        scene.render.image_settings.color_mode = 'RGBA' if self.transparent else 'RGB' # set color_mode
        scene.render.resolution_x = self.width # set resolution_x
        scene.render.resolution_y = self.height # set resolution_y
        scene.render.film_transparent = self.transparent # set transparent background
        scene.render.resolution_percentage = self.resolution_percentage # set resolution_percentage

        file_format = path.suffix[1:].upper()
        scene.render.image_settings.file_format = file_format # set file_format

        bpy.ops.render.render(write_still=True) # render the scene
