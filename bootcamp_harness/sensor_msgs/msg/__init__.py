import numpy as np
from numpy.typing import NDArray

MatLike = NDArray[np.generic]


class Image:
    def __init__(self, secret_internal_image: MatLike) -> None:
        self.secret_internal_image = secret_internal_image  # OpenCV image
