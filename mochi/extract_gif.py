from PIL import Image, ImageSequence
import os
import argparse
import math


def calculate_center_crop_box(original_width, original_height, target_aspect_ratio):
    target_ratio_w = target_aspect_ratio[0]
    target_ratio_h = target_aspect_ratio[1]
    crop_height_based_on_width = int(original_width * target_ratio_h / target_ratio_w)
    if crop_height_based_on_width <= original_height:
        crop_width = original_width
        crop_height = crop_height_based_on_width
    else:
        crop_width = int(original_height * target_ratio_w / target_ratio_h)
        crop_height = original_height

    offset_x = (original_width - crop_width) // 2
    offset_y = (original_height - crop_height) // 2
    crop_box = (
        offset_x,
        offset_y,
        offset_x + crop_width,
        offset_y + crop_height
    )
    return crop_box


def gif_to_c_header_with_aspect_crop(gif_path, output_header_path, target_width, target_height, var_name):
    frame_data_list = []
    frame_count = 0
    target_aspect_ratio = (target_width, target_height)
    try:
        with Image.open(gif_path) as im:
            original_width, original_height = im.size
            crop_box = calculate_center_crop_box(original_width, original_height, target_aspect_ratio)
            crop_left, crop_top, crop_right, crop_bottom = crop_box
            cropped_width = crop_right - crop_left
            cropped_height = crop_bottom - crop_top
            print(f"Original Size: {original_width}x{original_height}")
            print(f"Auto-Cropping to {cropped_width}x{cropped_height} (Aspect Ratio 16:13, centered).")
            for frame_index, frame in enumerate(ImageSequence.Iterator(im)):
                frame_rgb = frame.convert('RGB')
                cropped_frame = frame_rgb.crop(crop_box)
                resized_frame = cropped_frame.resize((target_width, target_height), Image.Resampling.LANCZOS)
                pixel_data = list(resized_frame.getdata())

                c_array_data = []
                for r, g, b in pixel_data:
                    rgb565 = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)
                    c_array_data.append(f"0x{rgb565 & 0xFF:02X}, 0x{(rgb565 >> 8) & 0xFF:02X}")

                frame_data_list.append(c_array_data)
            frame_data_list = frame_data_list[:min(len(frame_data_list), 10)]
            frame_count = len(frame_data_list)
            with open(output_header_path, 'w') as f:
                header_guard = os.path.basename(output_header_path).replace('.', '_').upper()
                f.write(f"#ifndef {header_guard}\n")
                f.write(f"#define {header_guard}\n\n")
                f.write(f"// GIF Animation Data\n")
                f.write(f"#define {var_name.upper()}_ANIM_FRAME_COUNT {frame_count}\n\n")
                frame_size_bytes = target_width * target_height * 2
                f.write(f"#define {var_name.upper()}_ANIM_FRAME_SIZE_BYTES {frame_size_bytes}\n\n")
                f.write(f"const unsigned char * const {var_name}s[{frame_count}];\n\n")

                for i, frame_data in enumerate(frame_data_list):
                    f.write(f"const unsigned char {var_name}_{i}[] = {{\n    ")
                    f.write(',\n    '.join([', '.join(frame_data[j:j + 16]) for j in range(0, len(frame_data), 16)]))
                    f.write("\n};\n\n")
                f.write(f"const unsigned char * const {var_name}s[{frame_count}] = {{\n")
                for i in range(frame_count):
                    f.write(f"    {var_name}_{i}")
                    if i < frame_count - 1:
                        f.write(",\n")
                    else:
                        f.write("\n")
                f.write("};\n\n")
                f.write(f"#endif // {header_guard}\n")

            print(f"✅ Successfully converted '{gif_path}' into '{output_header_path}'.")
            print(f"Frames: {frame_count}, Final Resolution: {target_width}x{target_height}")

    except FileNotFoundError:
        print(f"❌ Error: Input GIF file not found at '{gif_path}'")
        exit(1)
    except Exception as e:
        print(f"❌ An error occurred: {e}")
        exit(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Extracts GIF frames, center-crops to 16:13 aspect ratio, resizes to 160x130, and generates a C header file.",
        formatter_class=argparse.RawTextHelpFormatter
    )

    parser.add_argument(
        'input_gif',
        type=str,
        help="Path to the source GIF file (e.g., 'animation.gif')."
    )
    parser.add_argument(
        'output_header',
        type=str,
        help="Path for the destination C header file (e.g., 'image_data.h')."
    )

    parser.add_argument(
        '--width',
        type=int,
        default=160,
        help="Target image width (Default: 160)."
    )
    parser.add_argument(
        '--height',
        type=int,
        default=130,
        help="Target image height (Default: 130)."
    )
    parser.add_argument(
        '--var_name',
        type=str,
        default='anim_frame',
        help="header animation variable name",
    )

    args = parser.parse_args()

    gif_to_c_header_with_aspect_crop(
        args.input_gif,
        args.output_header,
        args.width,
        args.height,
        args.var_name,
    )
