import numpy as np

table = [
    ("Black", (0, 0, 0, 255)),
    ("White", (255, 255, 255, 255)),
    ("Red", (255, 0, 0, 255)),
    ("Lime", (0, 255, 0, 255)),
    ("Blue", (0, 0, 255, 255)),
    ("Yellow", (255, 255, 0, 255)),
    ("Cyan", (0, 255, 255, 255)),
    ("Magenta", (255, 0, 255, 255)),
    ("Silver", (192, 192, 192, 255)),
    ("Gray", (128, 128, 128, 255)),
    ("Maroon", (128, 0, 0, 255)),
    ("Olive", (128, 128, 0, 255)),
    ("Green", (0, 128, 0, 255)),
    ("Purple", (128, 0, 128, 255)),
    ("Teal", (0, 128, 128, 255)),
    ("Navy", (0, 0, 128, 255)),
]


for a, b in table:
    data = np.array(b)
    color_rgb = tuple(np.round(data / 255.0, 2))
    txt = """
inline jcc::Vec4 {color_name}() {{
    return jcc::Vec4{rgb};
}}
""".format(
        color_name=a.lower(),
        rgb=color_rgb
    )

    print txt
