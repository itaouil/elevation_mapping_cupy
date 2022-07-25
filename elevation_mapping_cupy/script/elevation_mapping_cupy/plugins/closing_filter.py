#
# Copyright (c) 2022, Takahiro Miki. All rights reserved.
# Licensed under the MIT license. See LICENSE file in the project root for details.
#
import cupy as cp
from typing import List
import cupyx.scipy.ndimage as ndimage

from .plugin_manager import PluginBase

class ClosingFilter(PluginBase):
    """Multidimensional binary closing with 
       the given structuring element in order
       to fill holes.

    ...

    Attributes
    ----------
    cell_n: int
        width and height of the elevation map.
    
    iterations: int
        number of times the filter has to applied

    input_later_name: str
        the later of the elevation map to apply it to
    """

    def __init__(self, cell_n: int = 100, size: int = 3, mode: str = "reflect", input_layer_name: str = "elevation", **kwargs):
        super().__init__()
        self.size = size
        self.mode = mode
        self.input_layer_name = input_layer_name

    def __call__(
        self,
        elevation_map: cp.ndarray,
        layer_names: List[str],
        plugin_layers: cp.ndarray,
        plugin_layer_names: List[str],
    ) -> cp.ndarray:
        if self.input_layer_name in layer_names:
            idx = layer_names.index(self.input_layer_name)
            h = elevation_map[idx]
        elif self.input_layer_name in plugin_layer_names:
            idx = plugin_layer_names.index(self.input_layer_name)
            h = plugin_layers[idx]
        else:
            print("layer name {} was not found. Using elevation layer.".format(self.input_layer_name))
            h = elevation_map[0]
        hm = ndimage.grey_closing(h, size=self.size, mode=self.mode)
        return hm
