import depth_pro
import numpy as np
import torch

config = depth_pro.DepthProConfig(
    patch_encoder_preset="dinov2l16_384",
    image_encoder_preset="dinov2l16_384",
    checkpoint_uri="../ml-depth-pro/checkpoints/depth_pro.pt",
    decoder_features=256,
    use_fov_head=True,
    fov_encoder_preset="dinov2l16_384",
)

model_global = None
transform_global = None


def fpx_from_f35(width: float, height: float, f_mm: float = 50) -> float:
    """Convert a focal length given in mm (35mm film equivalent) to pixels.
    Credit: Apple Depth Pro repository.
    """
    return f_mm * np.sqrt(width ** 2.0 + height ** 2.0) / np.sqrt(36 ** 2 + 24 ** 2)


def estimate_depth_map(image, image_resolution, image_focal_length, store_model_globally=False):
    device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")

    # Load model and preprocessing transform
    if store_model_globally:
        global model_global, transform_global
        if model_global is None:
            model_global, transform_global = depth_pro.create_model_and_transforms(config, device,
                                                                                   precision=torch.float16)
        model = model_global
        transform = transform_global
    else:
        model, transform = depth_pro.create_model_and_transforms(config, device, precision=torch.float16)
    model.eval()

    # Load and preprocess an image.
    f_px = fpx_from_f35(image_resolution[0], image_resolution[0], f_mm=image_focal_length)
    image = transform(image)

    # Run inference.
    prediction = model.infer(image, f_px=f_px)
    if not store_model_globally:
        del model
    depth = prediction["depth"].squeeze().cpu().numpy()

    return depth
